from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class Node:
    """Network node. One node can be fixed pressure (e.g., ambient = 0 Pa)."""
    id: int
    name: str = ""


@dataclass(frozen=True)
class Edge:
    """
    Directed edge from 'frm' to 'to'.

    Base resistance model (quadratic):
        Δp = r * Q|Q| - h
    where Δp = p(frm) - p(to)

    Fan (optional):
      fan_poly = (a3,a2,a1,a0) defines base fan curve h0(Q)
      speed_ratio s scales by affinity (simplified):
        h_s(Q) = s^2 * h0(Q/s)

    Terminal / Orifice (optional):
      terminal_orifice = (rho, Cd, Area)
      contributes additional quadratic resistance:
        r_term = rho / (2*(Cd*A)^2)

    Damper (optional):
      damper_k: quadratic coefficient at u=1 (fully open baseline)
      damper_u: opening ratio in (0, 1]; smaller -> higher resistance
      model:
        r_damper(u) = damper_k / u^2
      (If damper_k is None => no damper effect.)
    """
    id: int
    frm: int
    to: int
    r: float

    # constant head (assist from frm->to)
    h: float = 0.0

    # fan polynomial (a3,a2,a1,a0) or None
    fan_poly: Optional[Tuple[float, float, float, float]] = None
    speed_ratio: float = 1.0

    # terminal orifice (rho, Cd, Area) or None
    terminal_orifice: Optional[Tuple[float, float, float]] = None

    # damper parameters
    damper_k: Optional[float] = None
    damper_u: float = 1.0


@dataclass(frozen=True)
class Network:
    nodes: List[Node]
    edges: List[Edge]
    ref_node: int
