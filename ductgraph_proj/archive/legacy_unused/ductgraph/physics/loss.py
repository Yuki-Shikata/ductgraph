from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

from .friction import friction_factor_swamee_jain


def area_from_diameter(D: float) -> float:
    D = float(D)
    if D <= 0:
        raise ValueError("D must be > 0")
    return math.pi * (D ** 2) / 4.0


def r_from_K_total(*, K_total: float, rho: float, D: float) -> float:
    """
    Convert loss coefficient K_total (dimensionless, referenced to velocity head) into
    r in dp = r * Q|Q|, where Q is [m3/s] and dp is [Pa].
      dp = K * (rho/2) * v^2
      v = Q/A
      => dp = K * (rho/2) * (Q^2 / A^2) = [ K * rho/(2 A^2) ] * Q^2
      => r = K * rho/(2 A^2)
    """
    A = area_from_diameter(D)
    return float(K_total) * float(rho) / (2.0 * (A ** 2))


def r_from_straight_pipe(*, Q_design: float, rho: float, nu: float, eps: float, L: float, D: float) -> float:
    """
    Straight pipe friction converted to r (design-point fixed Re).
      dp = f*(L/D)*(rho/2)*v^2
      => K_fric = f*(L/D)
      => r = K_fric * rho/(2 A^2)
    """
    Q = float(Q_design)
    A = area_from_diameter(D)
    v = abs(Q) / A
    Re = (v * float(D)) / float(nu)
    f = friction_factor_swamee_jain(Re, eps=float(eps), D=float(D))
    K = f * (float(L) / float(D))
    return r_from_K_total(K_total=K, rho=rho, D=D)


def r_from_local_K(*, K: float, rho: float, D: float) -> float:
    return r_from_K_total(K_total=float(K), rho=rho, D=float(D))


def r_from_hood_dp_equivalent(*, dp_hood: float, Q_design: float) -> float:
    """
    Equivalent r for a hood modeled as constant dp at design point:
      dp = dp_hood ≈ r * Q^2  => r = dp_hood / Q_design^2
    (We treat it as Q^2 element; this is a pragmatic approximation for network solver.)
    """
    Q = float(Q_design)
    if Q <= 0:
        raise ValueError("Q_design must be > 0")
    return float(dp_hood) / (Q ** 2)


@dataclass(frozen=True)
class Segment:
    kind: str
    # geometry
    L: float = 0.0
    D: float = 0.0
    # local loss
    K: float = 0.0
    # hood
    dp: float = 0.0


def r_from_segments_designpoint(
    *,
    segments: Iterable[Segment],
    Q_design: float,
    rho: float,
    nu: float,
    eps: float,
    D_default: float,
) -> float:
    """
    Sum r contributions at the design point (Re fixed at design).
    - straight: uses Swamee–Jain f(Re, eps/D), converts to r
    - local K: converts to r
    - hood dp: equivalent r at design
    """
    r_total = 0.0
    for s in segments:
        kind = s.kind.lower().strip()
        D = float(s.D) if s.D else float(D_default)
        if kind in ("straight", "duct", "pipe"):
            if s.L <= 0 or D <= 0:
                raise ValueError(f"straight requires L>0 and D>0 (got L={s.L}, D={D})")
            r_total += r_from_straight_pipe(Q_design=Q_design, rho=rho, nu=nu, eps=eps, L=s.L, D=D)
        elif kind in ("elbow_90", "bend_90", "local", "k"):
            if s.K <= 0:
                raise ValueError(f"local K requires K>0 (got {s.K})")
            r_total += r_from_local_K(K=s.K, rho=rho, D=D)
        elif kind in ("hood",):
            if s.dp <= 0:
                raise ValueError(f"hood requires dp>0 (got {s.dp})")
            r_total += r_from_hood_dp_equivalent(dp_hood=s.dp, Q_design=Q_design)
        else:
            raise ValueError(f"unknown segment kind: {s.kind}")
    return float(r_total)
