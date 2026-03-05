from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Union

from .model import Network
from .solver_nodehead import SolveResult

Number = float
EffLike = Union[Number, Callable[[float, float, float], float]]
# callable signature: eta(Q_cmm, Pshaft_kw, speed_ratio) -> eta (0..1)


def m3s_to_cmm(q_m3s: float) -> float:
    """m^3/s -> m^3/min (CMM)"""
    return float(q_m3s) * 60.0


def _clamp_eta(x: float, *, name: str = "eta") -> float:
    x = float(x)
    if not (0.0 < x <= 1.0):
        raise ValueError(f"{name} must be in (0,1], got {x}")
    return x


def _eval_eta(eff: EffLike, *, Q_cmm: float, Pshaft_kw: float, s: float, name: str) -> float:
    if callable(eff):
        eta = float(eff(float(Q_cmm), float(Pshaft_kw), float(s)))
    else:
        eta = float(eff)
    return _clamp_eta(eta, name=name)


@dataclass(frozen=True)
class PowerPoly3:
    """
    Base condition (speed_ratio=1.0) shaft power curve:
      P0_kw(Q_cmm) = a*Q^3 + b*Q^2 + c*Q + d
    """
    a: float
    b: float
    c: float
    d: float
    f0_hz: float = 50.0  # metadata only

    def coeffs_at_speed(self, speed_ratio: float) -> tuple[float, float, float, float]:
        """
        Affinity transform for shaft power curve:
          Ps(Q) = s^3 * P0(Q/s)

        For cubic poly:
          a_s = a
          b_s = b*s
          c_s = c*s^2
          d_s = d*s^3
        """
        s = float(speed_ratio)
        return (
            float(self.a),
            float(self.b) * s,
            float(self.c) * s * s,
            float(self.d) * s * s * s,
        )

    def shaft_kw(self, Q_cmm: float, *, speed_ratio: float = 1.0) -> float:
        a, b, c, d = self.coeffs_at_speed(speed_ratio)
        Q = float(Q_cmm)
        return float(((a * Q + b) * Q + c) * Q + d)


@dataclass(frozen=True)
class FanPowerCurveResult:
    edge_id: int
    speed_ratio: float
    Q_m3s: float
    Q_cmm: float
    shaft_kw: float
    eta_total: float
    elec_kw: float


@dataclass(frozen=True)
class PowerCurveResult:
    fans: List[FanPowerCurveResult]
    shaft_total_kw: float
    elec_total_kw: float


def compute_electric_power_from_shaft_poly(
    net: Network,
    res: SolveResult,
    *,
    fan_edge_ids: Optional[Iterable[int]] = None,
    speed_ratio: float = 1.0,
    shaft_poly: PowerPoly3,
    eta_total: EffLike = 0.60,  # editable "general" default
) -> PowerCurveResult:
    """
    Compute:
      - shaft power via cubic poly (affinity-transformed by speed_ratio)
      - electrical power via eta_total

    Assumptions:
      - shaft_poly is defined for Q in CMM (m^3/min).
      - Use |Q| for power curve.
    """
    if not res.converged:
        raise ValueError("SolveResult not converged; power is not reliable.")

    # pick fan edges
    if fan_edge_ids is None:
        fan_ids = [e.id for e in net.edges if e.fan_poly is not None]
    else:
        wanted = set(int(x) for x in fan_edge_ids)
        fan_ids = [e.id for e in net.edges if (e.fan_poly is not None and e.id in wanted)]

    out: List[FanPowerCurveResult] = []
    shaft_sum = 0.0
    elec_sum = 0.0
    s = float(speed_ratio)

    for eid in fan_ids:
        Q_m3s = float(res.q[eid])          # signed
        Q_cmm = m3s_to_cmm(abs(Q_m3s))     # magnitude for power curve

        Pshaft = float(shaft_poly.shaft_kw(Q_cmm, speed_ratio=s))
        eta = _eval_eta(eta_total, Q_cmm=Q_cmm, Pshaft_kw=Pshaft, s=s, name="eta_total")
        Pelec = Pshaft / eta

        out.append(
            FanPowerCurveResult(
                edge_id=int(eid),
                speed_ratio=s,
                Q_m3s=Q_m3s,
                Q_cmm=Q_cmm,
                shaft_kw=Pshaft,
                eta_total=eta,
                elec_kw=float(Pelec),
            )
        )
        shaft_sum += Pshaft
        elec_sum += Pelec

    return PowerCurveResult(
        fans=out,
        shaft_total_kw=float(shaft_sum),
        elec_total_kw=float(elec_sum),
    )
