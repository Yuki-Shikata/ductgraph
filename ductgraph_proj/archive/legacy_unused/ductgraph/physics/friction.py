from __future__ import annotations

import math


def friction_factor_swamee_jain(Re: float, eps: float, D: float) -> float:
    """
    Darcy friction factor f.
    - Laminar: f = 64/Re
    - Turbulent: Swamee–Jain explicit approximation to Colebrook.
      f = 0.25 / [log10( eps/(3.7D) + 5.74/Re^0.9 )]^2
    Notes:
    - Re is evaluated at the design point and treated as fixed (as agreed).
    - eps is absolute roughness [m], D is inner diameter [m].
    """
    Re = float(Re)
    eps = float(eps)
    D = float(D)
    if D <= 0:
        raise ValueError("D must be > 0")
    if Re <= 0:
        raise ValueError("Re must be > 0")

    # laminar
    if Re < 2300.0:
        return 64.0 / Re

    term = (eps / (3.7 * D)) + (5.74 / (Re ** 0.9))
    if term <= 0:
        # should not happen for physical inputs
        term = 1e-12
    f = 0.25 / (math.log10(term) ** 2)
    return float(f)
