from __future__ import annotations

from typing import Dict

from .real_case_constants import *  # re-export stable physical/network constants
from .real_case_defaults import *  # re-export runtime defaults
from .real_case_network import (
    _compute_damper_k as _compute_damper_k_from_q,
    _compute_edge_r as _compute_edge_r_from_q,
    build_q_design,
    cmh_to_m3s,
    make_net as _make_net,
)


def choose_full_load_seed_edge(q_design_by_edge: Dict[int, float], r_by_edge: Dict[int, float]) -> int:
    best_eid = None
    best_dp = None
    for eid, q in q_design_by_edge.items():
        r = float(r_by_edge.get(eid, 0.0))
        dp = r * (q ** 2)
        if best_dp is None or dp > best_dp:
            best_dp = dp
            best_eid = eid
    assert best_eid is not None
    return int(best_eid)


# Deprecated alias for compatibility with older call sites.
def choose_maxload_edge(q_design_by_edge: Dict[int, float], r_by_edge: Dict[int, float]) -> int:
    return choose_full_load_seed_edge(q_design_by_edge, r_by_edge)


Q_DESIGN = build_q_design()


def _compute_edge_r() -> Dict[int, float]:
    return _compute_edge_r_from_q(Q_DESIGN)


def _compute_damper_k(
    *,
    damper_model: str = REALCASE_DAMPER_MODEL_DEFAULT,
    damper_gamma: float = REALCASE_DAMPER_GAMMA_DEFAULT,
) -> Dict[int, float]:
    return _compute_damper_k_from_q(
        Q_DESIGN,
        damper_model=damper_model,
        damper_gamma=damper_gamma,
    )


def make_net(
    *,
    damper_model: str = REALCASE_DAMPER_MODEL_DEFAULT,
    damper_gamma: float = REALCASE_DAMPER_GAMMA_DEFAULT,
    fan_poly: tuple[float, float, float, float] | None = None,
):
    return _make_net(
        damper_model=damper_model,
        damper_gamma=damper_gamma,
        q_design_by_edge=Q_DESIGN,
        fan_poly=fan_poly,
    )


_tmp = make_net(
    damper_model=REALCASE_DAMPER_MODEL_DEFAULT,
    damper_gamma=REALCASE_DAMPER_GAMMA_DEFAULT,
)
_r_by_eid = {e.id: float(e.r) for e in _tmp.edges if e.id in Q_DESIGN}
FULL_LOAD_SEED_EDGE = choose_full_load_seed_edge(Q_DESIGN, _r_by_eid)
MAXLOAD_EDGE = FULL_LOAD_SEED_EDGE  # deprecated alias

del _tmp, _r_by_eid
