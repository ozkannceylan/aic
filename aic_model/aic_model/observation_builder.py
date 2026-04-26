#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""ObservationBuilder — single-subscription observation conversion.

Converts ``aic_model_interfaces/msg/Observation`` messages from ``/observations``
into a plain Python dataclass that ``ApproachPolicy``, ``ClassicalInsertion``,
and ``SafetyWatchdog`` all read.

Design references:
  - ARCHITECTURE.md §3.2 — exact dataclass shape (every field is pinned there)
  - Lesson T-005 — DO NOT subscribe to /joint_states or /wrench separately;
    the adapter has already fused these into /observations
  - Backlog B-009 — Observation msg has no top-level header.stamp; freshness
    is checked on a specific field. We use ``center_image.header.stamp`` as
    the canonical observation timestamp because it is the slowest sensor stream
    and therefore the freshness-bottleneck in practice.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from aic_model_interfaces.msg import Observation as ObservationMsg


# Image encodings we accept. The sim publishes rgb8; bgr8 is supported in case
# upstream changes. Anything else: raise — we do not silently colour-swap.
_SUPPORTED_ENCODINGS = ("rgb8", "bgr8")


def _ros_pose_to_array(pose) -> np.ndarray:
    """geometry_msgs/Pose → ndarray(7,) [x, y, z, qx, qy, qz, qw]."""
    p = pose.position
    q = pose.orientation
    return np.array(
        [p.x, p.y, p.z, q.x, q.y, q.z, q.w],
        dtype=np.float64,
    )


def _ros_twist_to_array(twist) -> np.ndarray:
    """geometry_msgs/Twist → ndarray(6,) [vx, vy, vz, wx, wy, wz]."""
    l = twist.linear
    a = twist.angular
    return np.array(
        [l.x, l.y, l.z, a.x, a.y, a.z],
        dtype=np.float64,
    )


def _ros_wrench_to_array(wrench) -> np.ndarray:
    """geometry_msgs/Wrench → ndarray(6,) [fx, fy, fz, tx, ty, tz]."""
    f = wrench.force
    t = wrench.torque
    return np.array(
        [f.x, f.y, f.z, t.x, t.y, t.z],
        dtype=np.float64,
    )


def _ros_image_to_ndarray(image_msg) -> np.ndarray:
    """sensor_msgs/Image (rgb8|bgr8) → ndarray(H, W, 3) uint8.

    Mirrors tools/demo_recorder.py's resize_rgb_image conversion (without the
    resize). Native resolution is preserved; downstream code resizes when it
    packages the policy input dictionary (per ARCHITECTURE.md §3.2 — we keep
    full resolution in the dataclass so GeometricApproach can reuse it).
    """
    if image_msg.encoding not in _SUPPORTED_ENCODINGS:
        raise ValueError(
            f"Unsupported image encoding {image_msg.encoding!r}; "
            f"expected one of {_SUPPORTED_ENCODINGS}."
        )
    raw = np.frombuffer(image_msg.data, dtype=np.uint8)
    arr = raw.reshape(image_msg.height, image_msg.width, 3)
    if image_msg.encoding == "bgr8":
        arr = arr[..., ::-1]  # cheap channel flip; no copy if downstream
                              # consumes via fresh ndarray methods
    return arr


def _stamp_to_seconds(stamp) -> float:
    """builtin_interfaces/Time → float seconds (sec + nanosec * 1e-9)."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class Observation:
    """Policy-side observation, exactly the shape ARCHITECTURE.md §3.2 declares.

    All numeric fields are float64 except images (uint8) so SafetyWatchdog and
    classical phases reason about forces without precision loss. The 32-bit
    cast happens at the ACT-input boundary inside ApproachPolicy, not here.

    Frame conventions: every pose, velocity, and wrench is in ``base_link``;
    images are in their native sensor frames. See ARCHITECTURE.md §3.3.
    """

    timestamp: float                   # center_image.header.stamp in seconds
    left_image: np.ndarray             # (H, W, 3) uint8
    center_image: np.ndarray
    right_image: np.ndarray
    joint_positions: np.ndarray        # (6,)  first 6 entries of joint_states
    tcp_pose: np.ndarray               # (7,)  [x, y, z, qx, qy, qz, qw]
    tcp_velocity: np.ndarray           # (6,)  [vx, vy, vz, wx, wy, wz]
    raw_wrench: np.ndarray             # (6,)  [fx, fy, fz, tx, ty, tz]
    fts_tare_offset: np.ndarray        # (6,)  same shape as raw_wrench
    task_one_hot: np.ndarray           # (2,)  [1,0]=SFP, [0,1]=SC

    @property
    def compensated_wrench(self) -> np.ndarray:
        """Raw wrench minus live tare. The single source of truth for force.

        SafetyWatchdog (G4 @ 18 N), §5 PROBE (3 N), and §5 INSERT (17 N)
        thresholds all read this property. Downstream code must NOT subtract
        tare manually — central definition prevents one caller from forgetting.
        """
        return self.raw_wrench - self.fts_tare_offset


class ObservationBuilder:
    """Stateless converter from raw ROS msg → policy Observation dataclass.

    The orchestrator owns the ROS subscription. Every tick it calls
    ``ObservationBuilder.from_msg(latest_msg, task_one_hot)``. The builder
    performs no synchronization, no caching, no field validation beyond
    encoding — the adapter has already fused timestamps upstream (T4 evidence).

    ``task_one_hot`` is supplied by the orchestrator because it is latched
    from the action goal at trial start and is independent of sensor data
    (ARCHITECTURE.md §3.5). The builder does not know about action goals.
    """

    @staticmethod
    def from_msg(
        msg: ObservationMsg,
        task_one_hot: np.ndarray,
    ) -> Observation:
        cs = msg.controller_state
        joint_positions = np.array(msg.joint_states.position[:6], dtype=np.float64)
        return Observation(
            timestamp=_stamp_to_seconds(msg.center_image.header.stamp),
            left_image=_ros_image_to_ndarray(msg.left_image),
            center_image=_ros_image_to_ndarray(msg.center_image),
            right_image=_ros_image_to_ndarray(msg.right_image),
            joint_positions=joint_positions,
            tcp_pose=_ros_pose_to_array(cs.tcp_pose),
            tcp_velocity=_ros_twist_to_array(cs.tcp_velocity),
            raw_wrench=_ros_wrench_to_array(msg.wrist_wrench.wrench),
            fts_tare_offset=_ros_wrench_to_array(cs.fts_tare_offset.wrench),
            task_one_hot=task_one_hot,
        )

    @staticmethod
    def task_one_hot_from_plug_type(plug_type: str) -> np.ndarray:
        """Build the task_one_hot vector from the action goal's plug_type.

        Latched once per trial when the orchestrator accepts the action goal.
        Mapping: SFP → [1, 0], SC → [0, 1]. Anything else: raise — we do not
        silently fall back, because a wrong one-hot biases ACT predictions.
        """
        normalized = (plug_type or "").strip().lower()
        if normalized == "sfp":
            return np.array([1.0, 0.0], dtype=np.float32)
        if normalized == "sc":
            return np.array([0.0, 1.0], dtype=np.float32)
        raise ValueError(
            f"Unknown plug_type {plug_type!r}; expected 'sfp' or 'sc'."
        )


def observation_age_seconds(obs: Optional[Observation], now_seconds: float) -> float:
    """Age of an observation against ``now`` in seconds.

    Returns ``inf`` if ``obs`` is None so the freshness guard treats "no
    observation yet" as fully stale. Per ARCHITECTURE.md §3.4: > 200 ms skips
    the policy tick, > 2 s aborts the trial.
    """
    if obs is None:
        return float("inf")
    return max(0.0, now_seconds - obs.timestamp)
