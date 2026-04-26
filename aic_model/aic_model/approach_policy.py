#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""ApproachPolicy — SCAFFOLD ONLY (T22).

This is the placeholder shell that ``TrialOrchestrator`` calls during
``OrchestratorState.APPROACHING``. Real ACT inference lands in T22 round 2
(after T20 weights exist). For now ``step()`` returns the current TCP pose
unchanged so the orchestrator can be exercised end-to-end with mock or live
observations without any ACT dependency.

When the real implementation lands:
  1. Module-level ``import torch`` is FORBIDDEN — it fits inside the policy
     class ``__init__`` only (lessons T-015, validated by Intrinsic's
     troubleshooting docs commit #500 in fork).
  2. ``__init__`` may take 5-15 seconds to load weights; the engine's
     model-discovery budget is ~30 s, configure budget ~60 s. Stay well under.
  3. The class returns ``ApproachResult`` with ``final_pose`` pointing at the
     policy's last commanded pose. ``ClassicalInsertion`` starts from this
     pose's XY in APPROACH_REFINE.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from aic_model.observation_builder import Observation


@dataclass
class ApproachResult:
    """Outcome of one ApproachPolicy.run() invocation.

    success=True with final_pose set: handoff to ClassicalInsertion.
    success=False with reason: TrialOrchestrator can either retry (M3+) or
    fall back to GeometricApproach (M5 §7).
    """

    success: bool
    final_pose: Optional[np.ndarray]   # (7,) base_link pose, or None on failure
    reason: str = ""


class ApproachPolicy:
    """ACT-driven approach phase. Placeholder until weights are baked in.

    Constructor takes the parent rclpy node so logging and clock access work
    the same as any other Policy subclass. The orchestrator wires this up.
    """

    def __init__(self, parent_node) -> None:
        self._node = parent_node
        # T22 round 2 will load:
        #   self._model = ACTPolicy.from_pretrained(weights_path)
        #   self._chunk_overlap = 0.5
        #   self._device = "cuda" if torch.cuda.is_available() else "cpu"
        # For now nothing — scaffold only.
        self._node.get_logger().info(
            "ApproachPolicy SCAFFOLD ready (no ACT weights loaded yet)."
        )

    def run(
        self,
        observation: Observation,
        # NOTE: future signature also takes (move_robot, abort_check, deadline).
        # Kept narrow for the scaffold; orchestrator passes only the seed
        # observation today and gets a no-op pass-through pose back.
    ) -> ApproachResult:
        """Scaffold: pass current TCP pose straight through, claim success.

        Real implementation will run ACT inference in a chunk-overlap loop
        with temporal ensembling, publishing pose commands at ~20 Hz. See
        ARCHITECTURE.md §2 and §4.
        """
        return ApproachResult(
            success=True,
            final_pose=observation.tcp_pose.copy(),
            reason="scaffold pass-through",
        )
