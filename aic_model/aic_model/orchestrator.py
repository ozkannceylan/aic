#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""TrialOrchestrator — entry point for the M2+ submission.

Plain Python state machine (Enum + if/elif), single rclpy.Node (G3),
hard-coded 18 N soft cap via SafetyWatchdog (G4), no `/ground_truth_*`
subscriptions (G1). See ARCHITECTURE.md §6 for the design and growth path.

State graph (M2):

    IDLE → APPROACHING → INSERTING → COMPLETED
                                   ↘
                                    ABORTED  (any state can fall here)
                                       ↓
                                  retreat 10 cm Z, return success=False

The orchestrator is loaded by `aic_model.py` exactly the same way as
`CheatCode` / `WaveArm`: pass `policy:=aic_model.orchestrator` on the
`ros2 run aic_model aic_model` command line.
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from aic_control_interfaces.msg import MotionUpdate
from aic_model_interfaces.msg import Observation as ObservationMsg
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion

from aic_model.approach_policy import ApproachPolicy, ApproachResult
from aic_model.classical_insertion import ClassicalInsertion, InsertionResult
from aic_model.observation_builder import (
    Observation,
    ObservationBuilder,
    observation_age_seconds,
)
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model.safety_watchdog import SafetyWatchdog


# ---- Tunables (pinned in ARCHITECTURE.md §3.4 / §5.7) ----------------------

OBS_SKIP_AGE_S: float = 0.20         # > 200 ms: skip the policy tick
OBS_ABORT_AGE_S: float = 2.00        # > 2 s: abort the trial
ACTION_BUDGET_S: float = 30.0        # /insert_cable trial budget (engine side)
RETREAT_Z_M: float = 0.10            # § 5.7 retreat height
ORCHESTRATOR_TICK_S: float = 0.05    # 20 Hz idle-poll fallback
APPROACH_BACKOFF_S: float = 0.10     # wait for fresh observation before re-poll


# ---- State enum (G2 — plain Python) ----------------------------------------

class OrchestratorState(enum.Enum):
    IDLE = enum.auto()
    APPROACHING = enum.auto()
    INSERTING = enum.auto()
    COMPLETED = enum.auto()
    ABORTED = enum.auto()


@dataclass
class TrialState:
    """Per-trial bookkeeping. Reset at the start of every insert_cable goal."""

    state: OrchestratorState = OrchestratorState.IDLE
    abort_flag: bool = False
    abort_reason: str = ""
    task_one_hot: Optional[np.ndarray] = None
    last_observation: Optional[Observation] = None
    approach_final_pose: Optional[np.ndarray] = None
    approach_start_wall: float = 0.0
    insertion_result: Optional[InsertionResult] = None
    trial_start_wall: float = 0.0


# ---- Orchestrator ----------------------------------------------------------

class TrialOrchestrator(Policy):
    """Single-Node orchestrator. Loaded by aic_model.py via `policy:=aic_model.orchestrator`.

    Owns:
      - SafetyWatchdog (G4): background 20 Hz poll of compensated wrench.
      - ApproachPolicy: ACT-driven approach. SCAFFOLD in T22 round 1.
      - ClassicalInsertion: 5-phase impedance + spiral. SCAFFOLD in T22, real in T23.

    Does NOT own:
      - The `/observations` subscription (lives on the parent rclpy.Node — we
        just call get_observation()). Parent fuses upstream; we never sync.
      - The action server. The parent invokes ``insert_cable`` per goal.
      - Any non-aic_model ROS topic. G1 enforces /observations as the only
        sensor channel and the action goal as the only task-conditioning input.
    """

    def __init__(self, parent_node) -> None:
        super().__init__(parent_node)

        # Sub-policies. Constructors do NOT load weights or open files yet —
        # T15 / fork#500 docs require module-level imports + cheap __init__.
        self._approach = ApproachPolicy(parent_node)
        self._insertion = ClassicalInsertion(parent_node)

        # Per-trial mutable state. Re-initialized on every insert_cable call.
        self._trial: TrialState = TrialState()

        # Watchdog wires its breach callback into our trial-state mutator.
        self._watchdog = SafetyWatchdog(
            get_compensated_wrench=self._compensated_wrench_for_watchdog,
            on_breach=self._on_force_breach,
            logger=self.get_logger(),
        )

        self.get_logger().info(
            "TrialOrchestrator ready (M2 scaffold — ApproachPolicy + "
            "ClassicalInsertion are stubs)."
        )

    # ----- Policy interface --------------------------------------------------

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        """Entry point invoked by aic_model.py once per /insert_cable goal."""
        try:
            task_one_hot = ObservationBuilder.task_one_hot_from_plug_type(
                task.plug_type
            )
        except ValueError as exc:
            self.get_logger().error(f"insert_cable: {exc}; aborting trial.")
            send_feedback(f"abort: {exc}")
            return False

        self._reset_for_new_trial(task_one_hot)
        self.get_logger().info(
            f"insert_cable: plug_type={task.plug_type!r} "
            f"task_one_hot={task_one_hot.tolist()}; entering APPROACHING."
        )
        send_feedback(f"trial start: plug_type={task.plug_type}")

        self._watchdog.start()
        try:
            while self._trial.state not in (
                OrchestratorState.COMPLETED,
                OrchestratorState.ABORTED,
            ):
                if self._budget_expired():
                    self._set_abort("trial budget expired (30 s)")
                    break
                self._step(get_observation, move_robot, send_feedback)

            if self._trial.state is OrchestratorState.ABORTED:
                self._retreat(get_observation, move_robot, send_feedback)
                return False

            assert self._trial.state is OrchestratorState.COMPLETED
            send_feedback(
                f"completed: label={self._trial.insertion_result.label}"
                if self._trial.insertion_result is not None else "completed"
            )
            return True
        finally:
            self._watchdog.stop()

    # ----- State machine -----------------------------------------------------

    def _step(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> None:
        """One pass through the if/elif dispatcher.

        Each call either transitions state (cheap) or invokes a phase's
        blocking ``run()`` (long, but the phase polls ``self._trial.abort_flag``
        internally and returns when set). M3+ growth: more elif branches.
        See ARCHITECTURE.md §6.3-§6.5 for the canonical growth examples.
        """
        # Watchdog breach → abort takes precedence over state advancement.
        if self._trial.abort_flag and self._trial.state not in (
            OrchestratorState.ABORTED,
            OrchestratorState.COMPLETED,
        ):
            self._trial.state = OrchestratorState.ABORTED
            return

        # Refresh observation and apply freshness guard.
        obs = self._pull_fresh_observation(get_observation)
        if obs is None:
            # Either no observation yet or stale beyond skip threshold.
            time.sleep(APPROACH_BACKOFF_S)
            return

        state = self._trial.state

        if state is OrchestratorState.IDLE:
            self._trial.state = OrchestratorState.APPROACHING
            self._trial.approach_start_wall = time.monotonic()
            send_feedback("state: IDLE → APPROACHING")
            return

        if state is OrchestratorState.APPROACHING:
            result: ApproachResult = self._approach.run(obs)
            if self._trial.abort_flag:
                self._trial.state = OrchestratorState.ABORTED
                return
            if not result.success:
                self._set_abort(f"approach failed: {result.reason}")
                return
            self._trial.approach_final_pose = result.final_pose
            self._trial.state = OrchestratorState.INSERTING
            send_feedback(
                f"state: APPROACHING → INSERTING ({result.reason})"
            )
            return

        if state is OrchestratorState.INSERTING:
            # ClassicalInsertion owns its internal phase loop. We pass it the
            # raw-msg-getter wrapped in _pull_fresh_observation so the
            # SafetyWatchdog's compensated-wrench feed (which reads
            # self._trial.last_observation) stays current while we run.
            # task_one_hot is latched at trial start (§3.5).
            result_ins: InsertionResult = self._insertion.run(
                seed_observation=obs,
                task_one_hot=self._trial.task_one_hot,
                get_observation=lambda: self._pull_fresh_observation(get_observation),
                move_robot=move_robot,
                set_pose_target=self.set_pose_target,
                abort_check=lambda: self._trial.abort_flag,
                send_feedback=send_feedback,
            )
            if self._trial.abort_flag:
                self._trial.state = OrchestratorState.ABORTED
                return
            if not result_ins.success:
                self._set_abort(f"insertion failed: {result_ins.reason}")
                return
            self._trial.insertion_result = result_ins
            self._trial.state = OrchestratorState.COMPLETED
            send_feedback(
                f"state: INSERTING → COMPLETED ({result_ins.label})"
            )
            return

        # COMPLETED / ABORTED handled by the outer while() exit condition;
        # reaching here means an enum was added without a dispatch update.
        raise RuntimeError(
            f"Unhandled OrchestratorState in _step(): {state!r}"
        )

    # ----- Helpers -----------------------------------------------------------

    def _reset_for_new_trial(self, task_one_hot: np.ndarray) -> None:
        self._trial = TrialState(
            state=OrchestratorState.IDLE,
            task_one_hot=task_one_hot,
            trial_start_wall=time.monotonic(),
        )
        self._watchdog.reset()

    def _budget_expired(self) -> bool:
        return (
            time.monotonic() - self._trial.trial_start_wall
            > ACTION_BUDGET_S
        )

    def _pull_fresh_observation(
        self, get_observation: GetObservationCallback,
    ) -> Optional[Observation]:
        msg: Optional[ObservationMsg] = get_observation()
        if msg is None:
            return None
        try:
            obs = ObservationBuilder.from_msg(msg, self._trial.task_one_hot)
        except Exception as exc:
            self.get_logger().error(
                f"ObservationBuilder.from_msg failed: {exc}; skipping tick."
            )
            return None

        age = observation_age_seconds(obs, self._wall_now_seconds())
        if age > OBS_ABORT_AGE_S:
            self._set_abort(f"observation stale {age:.2f} s > {OBS_ABORT_AGE_S} s")
            return None
        if age > OBS_SKIP_AGE_S:
            # Stale but not catastrophic — orchestrator skips; a fresh msg
            # will arrive on the next tick.
            return None

        self._trial.last_observation = obs
        return obs

    def _wall_now_seconds(self) -> float:
        """Best-effort wall clock in seconds for freshness comparisons.

        Observation timestamps come from ROS time (sim or real); we compare
        with ``self.time_now()`` from the parent node's clock so sim-time
        runs (eval container) and wall-clock runs (live robot) both work.
        """
        ros_now = self.time_now()
        return float(ros_now.nanoseconds) * 1e-9

    def _set_abort(self, reason: str) -> None:
        if self._trial.abort_flag:
            return
        self._trial.abort_flag = True
        self._trial.abort_reason = reason
        self._trial.state = OrchestratorState.ABORTED
        self.get_logger().warning(f"ABORT: {reason}")

    def _on_force_breach(self, force_mag: float) -> None:
        self._set_abort(
            f"G4 watchdog: |F|={force_mag:.2f} N exceeded soft cap"
        )

    def _compensated_wrench_for_watchdog(self) -> Optional[np.ndarray]:
        """Watchdog calls this at 20 Hz from a background thread.

        Reading ``self._trial.last_observation`` is non-atomic but the GIL
        makes Python attribute reads atomic at instruction granularity, and
        we are reading a single reference. The dataclass is immutable per
        tick (we replace the whole object, never mutate fields), so a torn
        read is impossible.
        """
        obs = self._trial.last_observation
        if obs is None:
            return None
        return obs.compensated_wrench

    def _retreat(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> None:
        """Lift Z by RETREAT_Z_M from the current TCP pose. Best-effort.

        ARCHITECTURE.md §5.7: retreat is the orchestrator's job, not the
        insertion phase's. Single point of truth, fewer chances to miss a
        watchdog check inside duplicated retreat code.
        """
        msg = get_observation()
        if msg is None:
            send_feedback(f"retreat: no observation, abort_reason={self._trial.abort_reason}")
            return
        try:
            obs = ObservationBuilder.from_msg(msg, self._trial.task_one_hot)
        except Exception as exc:
            self.get_logger().error(f"_retreat: build observation failed: {exc}")
            return

        target_pose = obs.tcp_pose.copy()
        target_pose[2] += RETREAT_Z_M

        pose = Pose(
            position=Point(
                x=float(target_pose[0]),
                y=float(target_pose[1]),
                z=float(target_pose[2]),
            ),
            orientation=Quaternion(
                x=float(target_pose[3]),
                y=float(target_pose[4]),
                z=float(target_pose[5]),
                w=float(target_pose[6]),
            ),
        )
        try:
            self.set_pose_target(move_robot, pose)
            send_feedback(
                f"retreat: +{RETREAT_Z_M*100:.0f} cm Z; reason={self._trial.abort_reason}"
            )
        except Exception as exc:
            self.get_logger().error(f"_retreat: set_pose_target raised: {exc}")
