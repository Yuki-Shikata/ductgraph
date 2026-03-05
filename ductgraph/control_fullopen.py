from __future__ import annotations

from typing import Dict, List
from ductgraph.model import Network
from ductgraph.solver_nodehead import solve_node_head
from ductgraph.control_speed import with_speed_ratio
from ductgraph.control_damper import with_damper_angle


def eval_fullopen_flows(
    net: Network,
    *,
    speed_ratio: float,
    fan_edge_ids: List[int],
    cav_edge_ids: List[int],
    fixed_p: Dict[int, float],
    d: Dict[int, float],
    model: str = "sin",
    gamma: float = 1.0,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
) -> Dict[int, float]:
    """
    全CAVダンパを theta=90deg（全開）に固定したときの風量を返す
    """
    thetas = {eid: 90.0 for eid in cav_edge_ids}

    net_s = with_speed_ratio(net, speed_ratio, fan_edge_ids=fan_edge_ids)
    net_t = with_damper_angle(net_s, thetas, model=model, gamma=gamma)

    res = solve_node_head(
        net_t,
        d=d,
        fixed_p=fixed_p,
        fan_q_init=fan_q_init,
        fan_q_cap=fan_q_cap,
    )

    if not res.converged:
        raise RuntimeError("network did not converge at full open")

    return {eid: abs(res.q[eid]) for eid in cav_edge_ids}
