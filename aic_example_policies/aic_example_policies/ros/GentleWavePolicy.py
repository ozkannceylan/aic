#
#  Copyright (C) 2026 Ozkan Ceylan
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

"""GentleWavePolicy — M1 minimum-viable Policy for dummy submission.

Cartesian sinusoidal around the trial-start TCP pose. ±2 cm circular pattern
in the X-Y plane (base_link frame), Z held at start value, orientation latched
at start quaternion. Policy proves the observation -> command pipeline end to
end. Any score is acceptable — the M1 gate is demonstrated subscribe+act, not
performance.

References: ARCHITECTURE.md §2 (approach interface), §3 (observation schema),
§4 (action command). Lessons T-004 (ros2-daemon RMW lock) and T-005 (fused
/observations) apply — see tasks/lessons.md.

Guardrails (all apply):
- G1 GT-free: the framework's get_observation callback returns the fused
  /observations message only. No ground_truth topics are subscribed.
- G2 plain Python: state is four instance-local variables in insert_cable.
  No FSM or BT library.
- G3 single node: instantiated inside aic_model rclpy.LifecycleNode. No new
  nodes or threads.
- G4 18 N soft cap: not exercised. A 2 cm free-space sinusoidal will not
  reach the soft cap. SafetyWatchdog consumption of compensated_wrench is
  M2 work — computed by ObservationBuilder but not acted on in M1.
"""

import math

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration


class GentleWavePolicy(Policy):
    """±2 cm Cartesian sinusoidal around trial-start TCP pose."""

    # Wave parameters (§9 parameter provenance: G — gentle wave by spec).
    _AMPLITUDE_M = 0.02           # 2 cm translation envelope
    _FREQUENCY_HZ = 0.5           # 0.5 Hz, one full circle every 2 s
    _RUNTIME_SEC = 25.0           # Engine budget is 30 s/trial; 5 s margin
    _SLEEP_SEC = 0.05             # ~20 Hz loop, within §4 adaptive rate band

    # Impedance / feedback: we use Policy.set_pose_target's defaults, which
    # are the static values from WaveArm and match ARCHITECTURE.md §4's
    # decision to run static impedance through the whole qualification:
    #   stiffness    = [90, 90, 90, 50, 50, 50]  (expanded to 6x6 diag)
    #   damping      = [50, 50, 50, 20, 20, 20]  (expanded to 6x6 diag)
    #   wrench gains = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
    #   frame_id     = "base_link"
    #   mode         = MODE_POSITION
    # The 6-element list form is the helper's contract; hardcoding 36-float
    # matrices here would duplicate the np.diag(...).flatten() expansion.

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("GentleWavePolicy.__init__()")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(
            "GentleWavePolicy.insert_cable() enter. "
            f"plug_type={task.plug_type} port_type={task.port_type} "
            f"time_limit={task.time_limit}s"
        )
        send_feedback("gentle wave motion around trial-start TCP pose")

        # Latched state — populated on the first valid observation.
        trial_start_pose = None
        t0_sec = None
        obs_count = 0
        pub_count = 0

        start_time = self.time_now()
        timeout = Duration(seconds=self._RUNTIME_SEC)

        while (self.time_now() - start_time) < timeout:
            self.sleep_for(self._SLEEP_SEC)
            observation = get_observation()
            if observation is None:
                continue
            obs_count += 1

            # Observation.msg carries no top-level header — per-sensor headers
            # live inside each field. Center camera is the authoritative
            # timestamp per WaveArm convention.
            stamp = observation.center_image.header.stamp
            t_now = stamp.sec + stamp.nanosec / 1e9

            if trial_start_pose is None:
                src = observation.controller_state.tcp_pose
                # Defensive copy — the rclpy message object may be reused
                # by subsequent get_observation() calls.
                trial_start_pose = Pose(
                    position=Point(
                        x=src.position.x,
                        y=src.position.y,
                        z=src.position.z,
                    ),
                    orientation=Quaternion(
                        x=src.orientation.x,
                        y=src.orientation.y,
                        z=src.orientation.z,
                        w=src.orientation.w,
                    ),
                )
                t0_sec = t_now
                p = trial_start_pose.position
                q = trial_start_pose.orientation
                self.get_logger().info(
                    "Latched trial-start TCP pose: "
                    f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                    f"quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
                )

            # Circular X-Y oscillation around the latched start pose.
            t_elapsed = t_now - t0_sec
            omega = 2.0 * math.pi * self._FREQUENCY_HZ
            dx = self._AMPLITUDE_M * math.sin(omega * t_elapsed)
            dy = self._AMPLITUDE_M * math.sin(omega * t_elapsed + math.pi / 2.0)

            target_pose = Pose(
                position=Point(
                    x=trial_start_pose.position.x + dx,
                    y=trial_start_pose.position.y + dy,
                    z=trial_start_pose.position.z,
                ),
                orientation=Quaternion(
                    x=trial_start_pose.orientation.x,
                    y=trial_start_pose.orientation.y,
                    z=trial_start_pose.orientation.z,
                    w=trial_start_pose.orientation.w,
                ),
            )

            self.set_pose_target(move_robot=move_robot, pose=target_pose)
            pub_count += 1

            if pub_count % 10 == 0:
                self.get_logger().info(
                    f"obs={obs_count} pub={pub_count} "
                    f"cmd=[x,y,z]=[{target_pose.position.x:.4f}, "
                    f"{target_pose.position.y:.4f}, "
                    f"{target_pose.position.z:.4f}]"
                )

        self.get_logger().info(
            "GentleWavePolicy.insert_cable() exiting: "
            f"obs={obs_count} pub={pub_count}"
        )
        return True


# T10 manual smoke test (run by Ozkan, not Claude Code):
#
# Terminal 1: distrobox enter -r aic_eval
#             /entrypoint.sh ground_truth:=false start_aic_engine:=true
#
# Terminal 2 (host):
#   cd src/aic
#   pixi run ros2 run aic_model aic_model --ros-args \
#     -p use_sim_time:=true \
#     -p policy:=aic_example_policies.ros.GentleWavePolicy
#
# Terminal 3 (inside aic_eval; apply lesson T-003 first):
#   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
#   ros2 daemon stop && ros2 daemon start
#   ros2 topic hz /aic_controller/pose_commands
# Expected rate: ~20 Hz (matching _SLEEP_SEC = 0.05).
#
# Success criteria (M1 gate — live demonstration):
# - "GentleWavePolicy.__init__()" and "insert_cable() enter" logs present.
# - "Latched trial-start TCP pose: ..." logged once with finite values.
# - "obs=N pub=N cmd=[x,y,z]=..." at every 10th publication.
# - obs_count ≈ pub_count (1 publish per observation).
# - insert_cable exits cleanly with "obs=N pub=N" summary; no engine timeout.
