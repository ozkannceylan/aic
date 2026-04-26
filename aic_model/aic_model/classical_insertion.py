#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""ClassicalInsertion — SCAFFOLD ONLY (T23).

Placeholder for the impedance + spiral state machine described in
ARCHITECTURE.md §5. Real five-phase implementation (APPROACH_REFINE → ALIGN →
PROBE → INSERT → VERIFY) lands in T23.

The scaffold's ``run()`` returns ``InsertionResult(success=True, label="PARTIAL")``
without commanding any motion, so the orchestrator can dispatch through INSERTING
end-to-end during T22 smoke without forcing a real spiral search. Numeric
thresholds (3 N PROBE, 17 N INSERT jam, 8 s spiral timeout, etc.) are pinned
in ARCHITECTURE.md §5 and will be implemented there, not here.

When the real implementation lands:
  - Single-attempt policy (DEC-015). Retries belong at orchestrator level.
  - Every phase loop body checks ``abort_flag`` once per tick (G4 propagation).
  - Force thresholds read ``observation.compensated_wrench`` only — never
    ``raw_wrench`` directly (T-005 / observation_builder.compensated_wrench).
  - No RETREAT phase here; orchestrator handles retreat after we return
    (ARCHITECTURE.md §5.7).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from aic_model.observation_builder import Observation


@dataclass
class InsertionResult:
    """Outcome of one ClassicalInsertion.run() invocation.

    success=True, label in {"FULL", "PARTIAL"} → orchestrator → COMPLETED
    success=False, reason set                 → orchestrator → ABORTED + retreat

    Both FULL and PARTIAL map to ``bool success=True`` on the action result
    per InsertCable.action:9 (ARCHITECTURE.md §5.6). PARTIAL counts as success
    at M2 because the scoring table awards 38-50 points for partial insertion
    and returning success=False would discard them.
    """

    success: bool
    label: str = ""        # "FULL" / "PARTIAL" / "" on failure
    reason: str = ""       # populated when success=False
    final_z: Optional[float] = None   # final TCP z in base_link, for logging


class ClassicalInsertion:
    """Five-phase impedance + spiral insertion. Scaffold returns immediately.

    Constructor takes the parent rclpy node. T23 round will accept additional
    knobs (target_depth, spiral_radius_max, spiral_pitch, etc.) all sourced
    from ARCHITECTURE.md §9 — most flagged ``[estimated]`` until T24 validates.
    """

    def __init__(self, parent_node) -> None:
        self._node = parent_node
        self._node.get_logger().info(
            "ClassicalInsertion SCAFFOLD ready (no spiral logic yet)."
        )

    def run(
        self,
        observation: Observation,
        # NOTE: future signature also takes (move_robot, abort_check, send_feedback).
        # Scaffold ignores everything, claims trivial PARTIAL.
    ) -> InsertionResult:
        return InsertionResult(
            success=True,
            label="PARTIAL",
            reason="scaffold no-op",
            final_z=float(observation.tcp_pose[2]),
        )
