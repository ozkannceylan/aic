#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""Entry-point alias.

``aic_model.aic_model.AicModel`` discovers a policy by loading the module
named on the ROS parameter and looking up the class named ``module_name.split('.')[-1]``
inside it. To match that convention, the policy is loaded as
``policy:=aic_model.TrialOrchestrator`` — this file is the importable module,
and it re-exports the class of the same name from ``orchestrator.py``.

The actual implementation lives in ``orchestrator.py`` so the file size and
docstring stay reasonable; nothing here should grow beyond this re-export.
"""

from aic_model.orchestrator import TrialOrchestrator

__all__ = ["TrialOrchestrator"]
