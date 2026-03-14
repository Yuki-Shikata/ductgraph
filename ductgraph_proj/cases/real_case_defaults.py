from __future__ import annotations

from typing import List, Tuple

from .real_case_constants import EDGE_A, EDGE_C, EDGE_D

# Real-case tolerance defaults (equipment/operation side):
# - eps_under_rel: acceptance margin used by commissioning logic
# - tol_warn_rel: display threshold for WARN/UNDER split
EPS_UNDER_REL_DEFAULT = 0.01
TOL_WARN_REL_DEFAULT = 0.03

# Default damper model calibration for real_case network generation.
REALCASE_DAMPER_MODEL_DEFAULT = "expk"
REALCASE_DAMPER_GAMMA_DEFAULT = 3.0

# Default fan performance curve at base speed.
# dp[Pa] = a*Q^3 + b*Q^2 + c*Q + d, Q in m3/s.
REALCASE_FAN_POLY_DEFAULT = (-453.49632, 522.288, -257.04, 978.8)

# Default inverter operation bounds for real-case runner [Hz].
REALCASE_HZ_MIN_DEFAULT = 25.0
REALCASE_HZ_MAX_DEFAULT = 80.0

# Full-load opening target defaults.
REALCASE_THETA_CENTER_DEFAULT = 75.0
REALCASE_THETA_BAND_DEFAULT = 7.5

# Partial-load OFF-terminal control defaults.
REALCASE_THETA_OFF_DEFAULT = 0.0
REALCASE_OFF_DAMPER_U_DEFAULT = 1.0e-3

SCALING_CASES: List[Tuple[str, List[int]]] = [
    ("all", [EDGE_A, EDGE_C, EDGE_D]),
    ("a", [EDGE_A]),
    ("c", [EDGE_C]),
    ("d", [EDGE_D]),
    ("ac", [EDGE_A, EDGE_C]),
    ("ad", [EDGE_A, EDGE_D]),
    ("cd", [EDGE_C, EDGE_D]),
]
