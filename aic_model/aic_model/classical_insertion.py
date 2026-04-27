#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""ClassicalInsertion — ARCHITECTURE.md §5 five-phase impedance + spiral.

Phase machine (private, G2 plain Python Enum):

    APPROACH_REFINE → ALIGN → PROBE → INSERT → VERIFY → DONE
                                ↘                ↘
                                 ABORTED ←--------

Single-attempt (DEC-015). Retreat is owned by ``TrialOrchestrator`` (§5.7), not
this class. The class owns its internal tick loop and returns only on terminal
phases (DONE / ABORTED). Every per-tick body checks ``ctx.abort_check()`` before
issuing motion or reading further state — that is the G4 propagation contract.

Numeric thresholds are pinned in ARCHITECTURE.md §9; provenance code in the
trailing comment of each constant uses the §9 letters (M / E / G).

Forces are read via ``Observation.compensated_wrench`` only — never
``raw_wrench`` directly (T-005). The ObservationBuilder is the single source
of truth for the (raw - tare) subtraction.
"""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration

from aic_model.observation_builder import Observation


# ----- Tunables (ARCHITECTURE.md §9) ----------------------------------------
# Provenance: M=measured, E=estimated, G=guessed. T24 will validate each.

# §5.2 APPROACH_REFINE
APPROACH_REFINE_STANDOFF_M: float = 0.02         # G — 2 cm above seed Z
APPROACH_REFINE_POS_TOL_M: float = 3e-3          # G — 3 mm
APPROACH_REFINE_HOLD_S: float = 0.20             # G — 200 ms in tolerance
APPROACH_REFINE_TIMEOUT_S: float = 5.0           # G — local safety cap (not in §9)

# §5.3 ALIGN
ALIGN_TRANSITION_DEG: float = 1.0                # G — orient err < 1° → PROBE
ALIGN_SLERP_TRIGGER_DEG: float = 2.0             # G — orient err > 2° → slerp
ALIGN_TIMEOUT_S: float = 3.0                     # G — local safety cap

# §5.4 PROBE
PROBE_CONTACT_FZ_N: float = 3.0                  # G — above FTS noise floor
PROBE_DESCENT_M_PER_S: float = 0.5e-3            # G — 0.5 mm/s
PROBE_SPIRAL_RADIUS_MAX_M: float = 5e-3          # G — 0–5 mm
PROBE_SPIRAL_PITCH_M_PER_REV: float = 1e-3       # G — 1 mm/rev
PROBE_TIMEOUT_S: float = 8.0                     # E — ~25% CheatCode trial

# §5.5 INSERT
INSERT_JAM_FZ_N: float = 17.0                    # E — RES-003 6-8 N + G4 18 N cap
INSERT_TARGET_DEPTH_M: float = 0.02              # E — ~SFP port depth
INSERT_DESCENT_M_PER_S: float = 1.0e-3           # G — 1 mm/s ramp
INSERT_TIMEOUT_S: float = 30.0                   # G — 20 mm at 1 mm/s + slack

# §5.6 VERIFY
VERIFY_HOLD_S: float = 0.5                       # G — 500 ms settling
VERIFY_REBOUND_TOL_M: float = 1e-3               # G — 1 mm rebound
VERIFY_FULL_BOUNDARY_M: float = 2e-3             # E — 2 mm noise envelope

# §4 publish rates → tick periods
PROBE_TICK_S: float = 1.0 / 20.0                 # E — 20 Hz spiral
INSERT_TICK_S: float = 1.0 / 10.0                # E — 10 Hz gentle descent
DEFAULT_TICK_S: float = 1.0 / 20.0               # E — generic 20 Hz

# Default impedance (mirror Policy.set_pose_target signature; static for M2)
DEFAULT_STIFFNESS = [90.0, 90.0, 90.0, 50.0, 50.0, 50.0]
DEFAULT_DAMPING = [50.0, 50.0, 50.0, 20.0, 20.0, 20.0]


# ----- Phase enum (private, G2) ---------------------------------------------

class _Phase(enum.Enum):
    APPROACH_REFINE = enum.auto()
    ALIGN = enum.auto()
    PROBE = enum.auto()
    INSERT = enum.auto()
    VERIFY = enum.auto()
    DONE = enum.auto()
    ABORTED = enum.auto()


# ----- Public result --------------------------------------------------------

@dataclass
class InsertionResult:
    """Outcome of one ClassicalInsertion.run() invocation.

    success=True, label in {"FULL", "PARTIAL"} → orchestrator → COMPLETED.
    success=False, reason set                 → orchestrator → ABORTED + retreat.

    Both FULL and PARTIAL map to ``bool success=True`` on the action result
    per InsertCable.action:9 (ARCHITECTURE.md §5.6). PARTIAL counts as success
    at M2 because the scoring table awards 38-50 points for partial insertion
    and returning success=False would discard them.
    """

    success: bool
    label: str = ""        # "FULL" / "PARTIAL" / "" on failure
    reason: str = ""       # populated when success=False
    final_z: Optional[float] = None   # final TCP z in base_link, for logging


# ----- Internal context + phase state ---------------------------------------

GetObservationCallable = Callable[[], Optional[Observation]]
MoveRobotCallable = Callable[..., None]
SetPoseTargetCallable = Callable[..., None]
AbortCheck = Callable[[], bool]
SendFeedback = Callable[[str], None]


@dataclass
class _InsertionContext:
    """Bundle of callables passed in by orchestrator. Phase methods read this.

    ``get_observation`` returns a fresh ``Observation`` dataclass or None
    (already freshness-guarded by the orchestrator's ``_pull_fresh_observation``).
    Side effect: orchestrator's ``self._trial.last_observation`` is updated as
    part of that call, so the SafetyWatchdog's compensated-wrench feed stays
    current while we run.
    """

    task_one_hot: np.ndarray
    get_observation: GetObservationCallable
    move_robot: MoveRobotCallable
    set_pose_target: SetPoseTargetCallable
    abort_check: AbortCheck
    send_feedback: SendFeedback


@dataclass
class _PhaseState:
    """Mutable state passed between phases. Avoids long return tuples.

    ``seed_pose`` is the immutable trial entry pose (from ApproachPolicy
    handoff). ``entry_pose`` is captured fresh at each phase's start so
    PROBE / INSERT / VERIFY anchor on the post-REFINE TCP rather than the
    several-cm-off trial seed Z. Without this, PROBE would command a Z 2 cm
    below the post-REFINE TCP on its first tick → instant jam abort.
    """

    seed_pose: np.ndarray                   # (7,) trial entry, immutable
    entry_pose: np.ndarray                  # (7,) latest phase-entry obs
    contact_z: Optional[float] = None       # set by PROBE on contact
    insert_target_z: Optional[float] = None # set by INSERT
    final_z: Optional[float] = None         # set by INSERT on completion / VERIFY


# ----- ClassicalInsertion ---------------------------------------------------

class ClassicalInsertion:
    """Five-phase impedance + spiral insertion. ARCHITECTURE.md §5.

    Constructor takes the parent rclpy.Node (for logger + clock — sim-time
    aware via ``get_clock()``). Stateless across trials — every ``run()`` builds
    a fresh ``_PhaseState`` from the seed observation; nothing leaks between
    trials.
    """

    def __init__(self, parent_node) -> None:
        self._node = parent_node
        self._logger = parent_node.get_logger()
        self._logger.info(
            "ClassicalInsertion ready (5-phase impedance + spiral; "
            "ARCHITECTURE.md §5)."
        )

    # ----- Public entry point ----------------------------------------------

    def run(
        self,
        seed_observation: Observation,
        task_one_hot: np.ndarray,
        get_observation: GetObservationCallable,
        move_robot: MoveRobotCallable,
        set_pose_target: SetPoseTargetCallable,
        abort_check: AbortCheck,
        send_feedback: SendFeedback,
    ) -> InsertionResult:
        """Run APPROACH_REFINE → ALIGN → PROBE → INSERT → VERIFY → DONE.

        Returns:
            InsertionResult on terminal phase. The orchestrator handles retreat.
        """
        ctx = _InsertionContext(
            task_one_hot=task_one_hot,
            get_observation=get_observation,
            move_robot=move_robot,
            set_pose_target=set_pose_target,
            abort_check=abort_check,
            send_feedback=send_feedback,
        )
        seed_pose = seed_observation.tcp_pose.copy()
        ps = _PhaseState(seed_pose=seed_pose, entry_pose=seed_pose.copy())

        phase: _Phase = _Phase.APPROACH_REFINE
        message: str = ""

        # if/elif dispatch — G2 plain Python state machine.
        while phase not in (_Phase.DONE, _Phase.ABORTED):
            if phase is _Phase.APPROACH_REFINE:
                phase, message = self._phase_approach_refine(ctx, ps)
            elif phase is _Phase.ALIGN:
                phase, message = self._phase_align(ctx, ps)
            elif phase is _Phase.PROBE:
                phase, message = self._phase_probe(ctx, ps)
            elif phase is _Phase.INSERT:
                phase, message = self._phase_insert(ctx, ps)
            elif phase is _Phase.VERIFY:
                phase, message = self._phase_verify(ctx, ps)
            else:
                phase, message = _Phase.ABORTED, f"unhandled phase {phase!r}"

        if phase is _Phase.DONE:
            # message holds the FULL/PARTIAL label.
            return InsertionResult(
                success=True, label=message, final_z=ps.final_z,
            )
        return InsertionResult(
            success=False, reason=message, final_z=ps.final_z,
        )

    # ----- Phase 1: APPROACH_REFINE (§5.2) ---------------------------------

    def _phase_approach_refine(
        self, ctx: _InsertionContext, ps: _PhaseState,
    ) -> Tuple[_Phase, str]:
        """Hold pose 2 cm above seed Z; wait for settling within 3 mm for 200 ms.

        Without this, ACT terminal pose noise (several mm) biases the spiral
        center off-port and raises spiral-timeout risk (§5.2).
        """
        ctx.send_feedback("phase: APPROACH_REFINE")

        target_xyz = ps.seed_pose[:3].copy()
        target_xyz[2] += APPROACH_REFINE_STANDOFF_M
        target_quat = ps.seed_pose[3:].copy()
        target_pose = self._make_pose(target_xyz, target_quat)

        needed_ticks = max(1, int(round(APPROACH_REFINE_HOLD_S / DEFAULT_TICK_S)))
        in_tol_ticks = 0
        phase_start = self._sim_now_s()

        # Send the target once before entering the loop; controller starts moving.
        self._send_pose(ctx, target_pose)

        while True:
            if ctx.abort_check():
                return _Phase.ABORTED, "watchdog abort during APPROACH_REFINE"

            elapsed = self._sim_now_s() - phase_start
            if elapsed > APPROACH_REFINE_TIMEOUT_S:
                return _Phase.ABORTED, (
                    f"APPROACH_REFINE timeout after {elapsed:.2f}s "
                    f"(>{APPROACH_REFINE_TIMEOUT_S}s)"
                )

            obs = ctx.get_observation()
            if obs is None:
                self._sleep_tick(DEFAULT_TICK_S)
                continue

            pos_err = float(np.linalg.norm(obs.tcp_pose[:3] - target_xyz))
            if pos_err < APPROACH_REFINE_POS_TOL_M:
                in_tol_ticks += 1
                if in_tol_ticks >= needed_ticks:
                    ps.entry_pose = obs.tcp_pose.copy()
                    ctx.send_feedback(
                        f"APPROACH_REFINE: settled (pos_err={pos_err*1000:.2f} mm "
                        f"for {needed_ticks} ticks)"
                    )
                    return _Phase.ALIGN, ""
            else:
                in_tol_ticks = 0

            # Re-publish target every tick — controller is stable in 8-30 Hz band (§4).
            self._send_pose(ctx, target_pose)
            self._sleep_tick(DEFAULT_TICK_S)

    # ----- Phase 2: ALIGN (§5.3) -------------------------------------------

    def _phase_align(
        self, ctx: _InsertionContext, ps: _PhaseState,
    ) -> Tuple[_Phase, str]:
        """Orient toward nominal tool frame; immediate pass-through if err < 1°.

        M2 conservative: nominal_orientation = seed orientation, so orient_err
        is 0 by construction → immediate pass-through to PROBE. The slerp path
        is intentionally not exercised at M2 because we have no measured
        ground-truth nominal yet; a wrong nominal would actively hurt.

        TODO(M3 / T24): pin true nominal_orientation from CheatCode terminal
        quat measurement, then re-enable slerp for the > 2° branch.
        """
        ctx.send_feedback("phase: ALIGN (no-op, M2 seed-as-nominal)")

        if ctx.abort_check():
            return _Phase.ABORTED, "watchdog abort during ALIGN"

        # Single freshness-check tick so we observe at least once per phase.
        obs = ctx.get_observation()
        if obs is None:
            # If we cannot read even one obs in ALIGN, downstream phases fail too.
            self._sleep_tick(DEFAULT_TICK_S)

        # With seed-as-nominal, orient_err == 0 always → pass-through.
        return _Phase.PROBE, ""

    # ----- Phase 3: PROBE (§5.4) -------------------------------------------

    def _phase_probe(
        self, ctx: _InsertionContext, ps: _PhaseState,
    ) -> Tuple[_Phase, str]:
        """Archimedean spiral in XY with constant Z descent until Fz contact.

        On contact (|Fz| ≥ 3 N), capture contact_z and transition to INSERT.
        On 8 s timeout without contact, abort (orchestrator will retreat).
        """
        ctx.send_feedback("phase: PROBE")

        # Anchor the spiral at the post-REFINE TCP, NOT the trial seed pose.
        # See _PhaseState docstring for why.
        entry = self._wait_for_entry_obs(ctx)
        if entry is None:
            return _Phase.ABORTED, "PROBE: no fresh observation at phase entry"
        ps.entry_pose = entry.tcp_pose.copy()

        seed_xy = ps.entry_pose[:2].copy()
        seed_z = float(ps.entry_pose[2])
        seed_quat = ps.entry_pose[3:].copy()
        phase_start = self._sim_now_s()

        while True:
            if ctx.abort_check():
                return _Phase.ABORTED, "watchdog abort during PROBE"

            elapsed = self._sim_now_s() - phase_start
            if elapsed > PROBE_TIMEOUT_S:
                return _Phase.ABORTED, (
                    f"PROBE timeout after {elapsed:.2f}s without contact "
                    f"(>{PROBE_TIMEOUT_S}s)"
                )

            obs = ctx.get_observation()
            if obs is None:
                self._sleep_tick(PROBE_TICK_S)
                continue

            # Contact detection — magnitude of compensated Fz, sign-agnostic.
            fz = float(obs.compensated_wrench[2])
            if abs(fz) >= PROBE_CONTACT_FZ_N:
                ps.contact_z = float(obs.tcp_pose[2])
                ctx.send_feedback(
                    f"PROBE: contact at z={ps.contact_z:.4f} m, "
                    f"|Fz|={abs(fz):.2f} N"
                )
                return _Phase.INSERT, ""

            # Spiral target. Cap radius at PROBE_SPIRAL_RADIUS_MAX_M; once the
            # spiral has fully expanded, orbit at max radius until contact or
            # timeout.
            dx, dy = self._spiral_offset(elapsed)
            r = math.hypot(dx, dy)
            if r > PROBE_SPIRAL_RADIUS_MAX_M and r > 0:
                scale = PROBE_SPIRAL_RADIUS_MAX_M / r
                dx *= scale
                dy *= scale

            target_z = seed_z - PROBE_DESCENT_M_PER_S * elapsed
            target_xyz = np.array(
                [seed_xy[0] + dx, seed_xy[1] + dy, target_z],
                dtype=np.float64,
            )
            target_pose = self._make_pose(target_xyz, seed_quat)
            self._send_pose(ctx, target_pose)
            self._sleep_tick(PROBE_TICK_S)

    # ----- Phase 4: INSERT (§5.5) ------------------------------------------

    def _phase_insert(
        self, ctx: _InsertionContext, ps: _PhaseState,
    ) -> Tuple[_Phase, str]:
        """Slow Z ramp from contact_z down by INSERT_TARGET_DEPTH_M.

        Aborts on |Fz| ≥ 17 N (jam). Transitions to VERIFY when current Z
        reaches the target depth or stays within VERIFY_FULL_BOUNDARY_M of it.
        """
        if ps.contact_z is None:
            return _Phase.ABORTED, "INSERT entered without contact_z"

        # Capture phase entry obs so XY/quat anchor on post-PROBE TCP, not
        # the immutable trial seed (which may be cm away after REFINE).
        entry = self._wait_for_entry_obs(ctx)
        if entry is None:
            return _Phase.ABORTED, "INSERT: no fresh observation at phase entry"
        ps.entry_pose = entry.tcp_pose.copy()

        ps.insert_target_z = ps.contact_z - INSERT_TARGET_DEPTH_M
        seed_xy = ps.entry_pose[:2].copy()
        seed_quat = ps.entry_pose[3:].copy()
        phase_start = self._sim_now_s()

        ctx.send_feedback(
            f"phase: INSERT (contact_z={ps.contact_z:.4f}, "
            f"target_z={ps.insert_target_z:.4f})"
        )

        while True:
            if ctx.abort_check():
                return _Phase.ABORTED, "watchdog abort during INSERT"

            elapsed = self._sim_now_s() - phase_start
            if elapsed > INSERT_TIMEOUT_S:
                return _Phase.ABORTED, (
                    f"INSERT timeout after {elapsed:.2f}s "
                    f"(>{INSERT_TIMEOUT_S}s); "
                    f"target_z={ps.insert_target_z:.4f}"
                )

            obs = ctx.get_observation()
            if obs is None:
                self._sleep_tick(INSERT_TICK_S)
                continue

            fz = float(obs.compensated_wrench[2])
            if abs(fz) >= INSERT_JAM_FZ_N:
                return _Phase.ABORTED, (
                    f"INSERT jam at z={float(obs.tcp_pose[2]):.4f}, "
                    f"|Fz|={abs(fz):.2f} N >= {INSERT_JAM_FZ_N} N"
                )

            cur_z = float(obs.tcp_pose[2])
            if cur_z <= ps.insert_target_z + VERIFY_FULL_BOUNDARY_M:
                ps.final_z = cur_z
                ctx.send_feedback(
                    f"INSERT: target depth reached, final_z={cur_z:.4f}"
                )
                return _Phase.VERIFY, ""

            # Slow ramp: command Z one descent-step below current commanded Z,
            # bounded below by target_z. Controller chases the commanded Z;
            # impedance limits Fz, jam check above catches over-force.
            descent_step = INSERT_DESCENT_M_PER_S * INSERT_TICK_S
            cmd_z = max(ps.insert_target_z, cur_z - descent_step)
            target_pose = self._make_pose(
                np.array([seed_xy[0], seed_xy[1], cmd_z]), seed_quat,
            )
            self._send_pose(ctx, target_pose)
            self._sleep_tick(INSERT_TICK_S)

    # ----- Phase 5: VERIFY (§5.6) ------------------------------------------

    def _phase_verify(
        self, ctx: _InsertionContext, ps: _PhaseState,
    ) -> Tuple[_Phase, str]:
        """Hold final pose for 500 ms; classify FULL vs PARTIAL by final z.

        Returns (DONE, "FULL"|"PARTIAL") on success — the message slot carries
        the label, not a reason. (DONE, ...) is the only path that maps to
        success=True in run().
        """
        if ps.final_z is None or ps.insert_target_z is None:
            return _Phase.ABORTED, "VERIFY entered without final_z/target_z"

        # Hold at post-INSERT pose's XY/quat — most consistent with what the
        # controller is already chasing.
        entry = self._wait_for_entry_obs(ctx)
        if entry is None:
            return _Phase.ABORTED, "VERIFY: no fresh observation at phase entry"
        ps.entry_pose = entry.tcp_pose.copy()

        seed_xy = ps.entry_pose[:2].copy()
        seed_quat = ps.entry_pose[3:].copy()
        hold_pose = self._make_pose(
            np.array([seed_xy[0], seed_xy[1], ps.final_z]), seed_quat,
        )

        ctx.send_feedback(f"phase: VERIFY (hold {VERIFY_HOLD_S*1000:.0f} ms)")

        phase_start = self._sim_now_s()
        z_samples: list[float] = []

        while (self._sim_now_s() - phase_start) < VERIFY_HOLD_S:
            if ctx.abort_check():
                return _Phase.ABORTED, "watchdog abort during VERIFY"

            obs = ctx.get_observation()
            if obs is None:
                self._sleep_tick(DEFAULT_TICK_S)
                continue

            z_samples.append(float(obs.tcp_pose[2]))
            self._send_pose(ctx, hold_pose)
            self._sleep_tick(DEFAULT_TICK_S)

        if not z_samples:
            return _Phase.ABORTED, "VERIFY no fresh observations within hold window"

        # Update final_z to last observed Z (most accurate).
        ps.final_z = z_samples[-1]

        # Rebound check: if Z swung > 1 mm during hold, plug is bouncing → PARTIAL.
        rebound = max(z_samples) - min(z_samples)
        if rebound > VERIFY_REBOUND_TOL_M:
            ctx.send_feedback(
                f"VERIFY: rebound {rebound*1000:.2f} mm > "
                f"{VERIFY_REBOUND_TOL_M*1000:.1f} mm → PARTIAL"
            )
            return _Phase.DONE, "PARTIAL"

        # Distance-from-target classifier. positive = short of target depth.
        distance_short = ps.final_z - ps.insert_target_z
        label = self._classify_full_partial(distance_short)
        ctx.send_feedback(
            f"VERIFY: final_z={ps.final_z:.4f}, target_z={ps.insert_target_z:.4f}, "
            f"distance_short={distance_short*1000:.2f} mm → {label}"
        )
        return _Phase.DONE, label

    # ----- Pure helpers (testable without ROS) -----------------------------

    @staticmethod
    def _spiral_offset(elapsed_s: float) -> Tuple[float, float]:
        """Archimedean spiral: r(θ) = (pitch / 2π) · θ.

        Pace ω chosen so r reaches PROBE_SPIRAL_RADIUS_MAX_M at 80% of the
        PROBE_TIMEOUT_S budget — leaves 20% of the budget for orbiting at max
        radius before the timeout fires.

        Returns (dx, dy) in metres. Caller clips r at PROBE_SPIRAL_RADIUS_MAX_M.
        """
        # θ_max = r_max · 2π / pitch
        theta_max = (
            PROBE_SPIRAL_RADIUS_MAX_M * 2.0 * math.pi
            / PROBE_SPIRAL_PITCH_M_PER_REV
        )
        full_radius_at_s = PROBE_TIMEOUT_S * 0.8
        omega = theta_max / full_radius_at_s   # rad/s
        theta = omega * elapsed_s
        r = (PROBE_SPIRAL_PITCH_M_PER_REV / (2.0 * math.pi)) * theta
        return r * math.cos(theta), r * math.sin(theta)

    @staticmethod
    def _classify_full_partial(distance_short_m: float) -> str:
        """FULL if final_z is within VERIFY_FULL_BOUNDARY_M of target; else PARTIAL.

        ``distance_short_m`` is positive when final_z is above target (short of
        full insertion). Negative or zero values mean we reached or overshot
        the target, which is FULL. The classifier is total — there is no
        "unknown" state.
        """
        if distance_short_m <= VERIFY_FULL_BOUNDARY_M:
            return "FULL"
        return "PARTIAL"

    @staticmethod
    def _quaternion_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
        """Angle between two unit quaternions in degrees, in [0, 180].

        Uses |dot(q1, q2)| to handle the q ≡ -q double-cover. Robust to
        numerical drift slightly outside [-1, 1].
        """
        d = float(np.clip(abs(np.dot(q1, q2)), 0.0, 1.0))
        return math.degrees(2.0 * math.acos(d))

    # ----- Internal plumbing ------------------------------------------------

    @staticmethod
    def _make_pose(xyz: np.ndarray, quat: np.ndarray) -> Pose:
        return Pose(
            position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
            orientation=Quaternion(
                x=float(quat[0]), y=float(quat[1]),
                z=float(quat[2]), w=float(quat[3]),
            ),
        )

    @staticmethod
    def _send_pose(ctx: _InsertionContext, pose: Pose) -> None:
        """Issue a MotionUpdate via orchestrator's bound Policy.set_pose_target.

        Centralising through ``Policy.set_pose_target`` keeps the default
        impedance / damping / wrench-feedback constants in one place. Any
        future per-phase impedance modulation (B-002, M3+) lands there too.
        """
        ctx.set_pose_target(ctx.move_robot, pose)

    def _sim_now_s(self) -> float:
        """Sim-time-aware wall clock. Eval container runs sim time, host runs
        wall time; both work because we use the node's clock."""
        return float(self._node.get_clock().now().nanoseconds) * 1e-9

    def _sleep_tick(self, period_s: float) -> None:
        """Sim-time-aware sleep using the node's clock (mirrors Policy.sleep_for).

        Falls back to wall-clock ``time.sleep`` if the clock raises (e.g. in
        unit-test contexts with a stub clock that does not implement
        sleep_for). Tests that need deterministic time injection should patch
        ``self._node.get_clock()`` directly.
        """
        try:
            self._node.get_clock().sleep_for(Duration(seconds=period_s))
        except Exception:
            time.sleep(period_s)

    def _wait_for_entry_obs(
        self, ctx: _InsertionContext, max_retries: int = 40,
    ) -> Optional[Observation]:
        """Bounded retry loop for the first observation at phase entry.

        40 retries at DEFAULT_TICK_S = 2 s — enough for one freshness wraparound
        even on a busy eval bus. Returns None on abort or budget exhaustion;
        callers must treat None as a phase-abort condition.
        """
        for _ in range(max_retries):
            if ctx.abort_check():
                return None
            obs = ctx.get_observation()
            if obs is not None:
                return obs
            self._sleep_tick(DEFAULT_TICK_S)
        return None
