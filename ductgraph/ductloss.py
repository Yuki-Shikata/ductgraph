from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence, Tuple, Optional, Literal

# -------------------------
# Units helpers
# -------------------------
def cmh_to_m3s(q_cmh: float) -> float:
    return float(q_cmh) / 3600.0

def m3s_to_cmh(q_m3s: float) -> float:
    return float(q_m3s) * 3600.0

def area_from_diameter(D_m: float) -> float:
    D = float(D_m)
    if D <= 0:
        raise ValueError("D must be > 0")
    return math.pi * (D * D) / 4.0

# -------------------------
# Friction factor (Swamee-Jain)
# -------------------------
def friction_factor_swamee_jain(
    *,
    Re: float,
    D_m: float,
    eps_m: float,
    Re_laminar: float = 2300.0,
) -> float:
    """
    Darcy friction factor.
    - Laminar: f = 64/Re
    - Turbulent: Swamee-Jain:
        f = 0.25 / [log10( eps/(3.7D) + 5.74/Re^0.9 )]^2
    """
    Re = float(Re)
    D = float(D_m)
    eps = float(eps_m)

    if D <= 0:
        raise ValueError("D_m must be > 0")
    if Re <= 0:
        # treat as very low flow -> extremely high friction; but caller should avoid Q=0
        return 1.0

    if Re < Re_laminar:
        return 64.0 / Re

    # clamp to avoid log issues
    term = (eps / (3.7 * D)) + (5.74 / (Re ** 0.9))
    term = max(term, 1e-16)
    return 0.25 / (math.log10(term) ** 2)

# -------------------------
# Segment models
# -------------------------
@dataclass(frozen=True)
class Straight:
    L_m: float
    D_m: float

@dataclass(frozen=True)
class LocalK:
    K: float
    # Optional diameter override for area (e.g., elbow at a certain D)
    D_m: Optional[float] = None

Segment = Tuple[str, float, float]  # legacy placeholder if needed

def equivalent_r_from_f_and_K(
    *,
    rho: float,
    D_m: float,
    fL_over_D: float,
    K_sum: float,
) -> float:
    """
    For a duct element (same diameter for A):
      Δp = (fL/D + ΣK) * (rho/(2A^2)) * Q^2
    In solver model: Δp = r * Q|Q|  =>  r = (fL/D + ΣK) * rho/(2A^2)
    """
    A = area_from_diameter(D_m)
    coeff = float(fL_over_D) + float(K_sum)
    if coeff < 0:
        raise ValueError("fL_over_D + K_sum must be >= 0")
    return coeff * float(rho) / (2.0 * (A ** 2))

def r_from_segments_design_point(
    *,
    Q_m3s: float,
    rho: float,
    nu_m2s: float,
    roughness_m: float,
    straight_runs: Sequence[Straight],
    locals_K: Sequence[LocalK],
    safety_factor: float = 1.0,  # >=1.0 : 抵抗を安全側に寄せる
) -> float:
    """
    Compute equivalent r for a series of segments at a design flow Q.
    - Straight: Darcy–Weisbach with Swamee–Jain (via Re)
    - Local: K * rho/(2A^2) is included as part of r via K_sum

    NOTE: The resulting r is "fixed" (design-point-based) but physically consistent
    because f depends on Re(Q). If Q changes, f should change; we accept this
    as a design-point approximation (and you asked for safety-side anyway).
    """
    Q = float(Q_m3s)
    if Q <= 0:
        raise ValueError("Q_m3s must be > 0 for design-point friction factor")

    rho = float(rho)
    nu = float(nu_m2s)
    eps = float(roughness_m)

    # Accumulate fL/D per straight run (each run may have its own D)
    fL_over_D_sum = 0.0
    # Accumulate K (dimensionless), but note: if D differs, strictly K should map
    # to each diameter's A. Here we assume locals belong to the "main" diameter of the element.
    # If you need mixed diameters, split into multiple r blocks per diameter.
    K_sum = 0.0

    # Choose a reference diameter: if multiple, we compute f per run and later require a single D.
    # To keep solver-simple, enforce single D for one Edge.r block.
    Ds = {float(s.D_m) for s in straight_runs}
    Ds2 = {float(k.D_m) for k in locals_K if k.D_m is not None}
    Ds_all = Ds.union(Ds2)
    if len(Ds_all) == 0:
        raise ValueError("At least one diameter must be provided")
    if len(Ds_all) != 1:
        raise ValueError(
            f"Mixed diameters in one Edge.r block not supported (found {sorted(Ds_all)}). "
            f"Split into multiple Edges or compute separate r blocks."
        )
    D = Ds_all.pop()

    A = area_from_diameter(D)
    V = Q / A
    Re = abs(V) * D / nu

    # Use a single friction factor at design point for this Edge block
    f = friction_factor_swamee_jain(Re=Re, D_m=D, eps_m=eps)

    for s in straight_runs:
        L = float(s.L_m)
        if L < 0:
            raise ValueError("Straight.L_m must be >= 0")
        fL_over_D_sum += f * (L / D)

    for lk in locals_K:
        K = float(lk.K)
        if K < 0:
            raise ValueError("Local K must be >= 0")
        K_sum += K

    r = equivalent_r_from_f_and_K(rho=rho, D_m=D, fL_over_D=fL_over_D_sum, K_sum=K_sum)

    sf = float(safety_factor)
    if sf < 1.0:
        raise ValueError("safety_factor must be >= 1.0")
    return r * sf

def r_from_const_dp_at_design(
    *,
    dp_pa: float,
    Q_m3s_design: float,
    safety_factor: float = 1.0,
) -> float:
    """
    Convert constant pressure loss (e.g. hood dp) into equivalent r at design point:
      dp = r * Q^2  =>  r = dp / Q^2
    Safety factor >=1 increases resistance (conservative).
    """
    dp = float(dp_pa)
    Q = float(Q_m3s_design)
    if dp < 0:
        raise ValueError("dp_pa must be >= 0")
    if Q <= 0:
        raise ValueError("Q_m3s_design must be > 0")
    sf = float(safety_factor)
    if sf < 1.0:
        raise ValueError("safety_factor must be >= 1.0")
    return sf * dp / (Q * Q)

def interp_table(x: float, table: Sequence[Tuple[float, float]]) -> float:
    """
    Piecewise-linear interpolation for a monotonically increasing x table.
    table: [(x0,y0), (x1,y1), ...]
    """
    if not table:
        raise ValueError("table empty")
    xx = float(x)
    pts = [(float(a), float(b)) for a, b in table]
    pts.sort(key=lambda t: t[0])

    if xx <= pts[0][0]:
        return pts[0][1]
    if xx >= pts[-1][0]:
        return pts[-1][1]

    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        if x0 <= xx <= x1:
            t = 0.0 if x1 == x0 else (xx - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return pts[-1][1]

def tee_K_from_ratio(
    *,
    q_branch: float,
    q_straight: float,
    table_branch: Sequence[Tuple[float, float]],
    table_straight: Sequence[Tuple[float, float]],
) -> Tuple[float, float, float]:
    """
    Compute Tee K at design ratio r = Q_branch / Q_parent, where Q_parent = Q_branch + Q_straight.
    Returns: (r_ratio, K_branch, K_straight)
    """
    qb = float(q_branch)
    qs = float(q_straight)
    qp = qb + qs
    if qp <= 0:
        raise ValueError("q_branch + q_straight must be > 0")
    r = qb / qp
    # clamp to [0,1]
    r = max(0.0, min(1.0, r))
    Kb = interp_table(r, table_branch)
    Ks = interp_table(r, table_straight)
    return r, Kb, Ks
