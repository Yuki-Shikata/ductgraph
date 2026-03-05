from __future__ import annotations

from dataclasses import replace
from typing import Dict, Iterable, Tuple, Optional, List

from ductgraph.model import Network, Edge
from ductgraph.solver_nodehead import solve_node_head, SolveResult
from ductgraph.control_damper import with_damper_angle


def with_speed_ratio(net: Network, speed_ratio: float, *, fan_edge_ids: Optional[Iterable[int]] = None) -> Network:
    """
    Return a new Network where fan edges' speed_ratio are overwritten.

    If fan_edge_ids is None:
      apply to all edges that have fan_poly != None
    else:
      apply to edges whose id is in fan_edge_ids (and have fan_poly != None)
    """
    s = float(speed_ratio)
    fan_id_set = None if fan_edge_ids is None else set(int(x) for x in fan_edge_ids)

    new_edges: List[Edge] = []
    for e in net.edges:
        if e.fan_poly is not None:
            if fan_id_set is None or e.id in fan_id_set:
                new_edges.append(replace(e, speed_ratio=s))
            else:
                new_edges.append(e)
        else:
            new_edges.append(e)

    return Network(nodes=net.nodes, edges=new_edges, ref_node=net.ref_node)


def tune_speed_for_maxload(
    net: Network,
    *,
    fan_edge_ids: Optional[Iterable[int]],
    maxload_edge_id: int,
    q_design: float,
    theta_center: float = 75.0,
    theta_band: float = 10.0,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    s_min: float = 0.2,
    s_max: float = 1.8,
    tol_q: float = 1e-3,
    max_iter: int = 40,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 50.0,
    thetas_fixed: Optional[Dict[int, float]] = None,
) -> Tuple[float, float, SolveResult]:
    """
    Commissioning-style speed tuning (VFD):

    Goal:
      - Fix max-load damper at theta_center (e.g., 75 deg)
      - Find speed_ratio s such that flow on maxload_edge_id equals q_design

    Also returns:
      - q_lo = Q at theta_center - theta_band (e.g., 65deg)
      - q_hi = Q at theta_center + theta_band (e.g., 85deg)
    so you can verify "±10deg has control authority" around the design point.

    Returns: (best_speed_ratio, (q_at_65, q_at_85, last_result))
    """
    if fixed_p is None:
        fixed_p = {net.ref_node: 0.0}
    if d is None:
        d = {}

    theta_lo = theta_center - theta_band
    theta_hi = theta_center + theta_band

    # evaluate Q at a given (speed_ratio, theta_at_maxload)
    def eval_q(speed: float, theta: float) -> Tuple[float, SolveResult]:
        net_s = with_speed_ratio(net, speed, fan_edge_ids=fan_edge_ids)
        ths = {} if thetas_fixed is None else dict(thetas_fixed)
        ths[maxload_edge_id] = theta
        net_st = with_damper_angle(net_s, ths, model=model, gamma=gamma)
        res = solve_node_head(net_st, d=d, fixed_p=fixed_p, fan_q_init=fan_q_init, fan_q_cap=fan_q_cap)
        return float(res.q[maxload_edge_id]), res

    # bracket in speed dimension using theta_center
    q_smin, res = eval_q(s_min, theta_center)
    q_smax, res2 = eval_q(s_max, theta_center)

    # ensure monotonic bracket direction (expect q increases with speed)
    # if it doesn't, we still proceed by sorting
    if q_smin > q_smax:
        s_min, s_max = s_max, s_min
        q_smin, q_smax = q_smax, q_smin
        res, res2 = res2, res

    # if target is outside reachable range, clamp speed
    if q_design <= q_smin:
        best_s = float(s_min)
        q_mid, res_mid = q_smin, res
    elif q_design >= q_smax:
        best_s = float(s_max)
        q_mid, res_mid = q_smax, res2
    else:
        lo, hi = float(s_min), float(s_max)
        q_mid, res_mid = 0.0, res2
        for _ in range(max_iter):
            mid = 0.5 * (lo + hi)
            q_mid, res_mid = eval_q(mid, theta_center)
            if abs(q_mid - q_design) < tol_q and res_mid.converged:
                lo = hi = mid
                break
            if q_mid < q_design:
                lo = mid
            else:
                hi = mid
        best_s = 0.5 * (lo + hi)
        q_mid, res_mid = eval_q(best_s, theta_center)

    # compute authority check at theta_lo/theta_hi with chosen speed
    q_lo, _ = eval_q(best_s, theta_lo)
    q_hi, _ = eval_q(best_s, theta_hi)

    return best_s, (q_lo, q_hi, res_mid)
