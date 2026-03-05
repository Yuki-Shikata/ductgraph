from __future__ import annotations

import math
from dataclasses import replace
from typing import Dict, Iterable, Tuple, Optional, List

from ductgraph.model import Network, Edge
from ductgraph.solver_nodehead import solve_node_head, SolveResult


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def angle_to_u(theta_deg: float, *, model: str = "sin", gamma: float = 1.0) -> float:
    """
    Convert damper angle (deg) to opening ratio u in (0, 1].

    theta_deg:
      - 0 deg  : fully closed (minimum opening)
      - 90 deg : fully open

    model:
      - "sin":    u = sin(theta) ** gamma
      - "linear": u = (theta/90) ** gamma

      - "exp":    (LEGACY) u = exp(-beta * (90 - theta)/90), beta=gamma
                  -> implies K(theta) multiplier ~= exp(2*beta*(90-theta)/90)
                  (kept for backward compatibility)

      - "expk":   (RECOMMENDED) K(theta) multiplier is exponential:
                    Kmult(theta) = exp(beta * (90 - theta)/90), beta=gamma
                  Implemented via u so that K/u^2 = K*Kmult:
                    u = exp(-0.5*beta*(90-theta)/90)

    gamma:
      - sin/linear: exponent
      - exp/expk:   beta (strength). Larger => stronger throttling when closing.
    """
    th = _clamp(float(theta_deg), 0.0, 90.0)

    if model == "sin":
        u0 = math.sin(math.radians(th))
        u = max(u0, 1e-6) ** float(gamma)

    elif model == "linear":
        u0 = th / 90.0
        u = max(u0, 1e-6) ** float(gamma)

    elif model == "exp":
        beta = float(gamma)
        phi_close = 90.0 - th  # 0=open, 90=closed
        if abs(beta) < 1e-12:
            u = 1.0
        else:
            u = math.exp(-beta * (phi_close / 90.0))
        u = _clamp(u, 1e-6, 1.0)

    elif model == "expk":
        beta = float(gamma)
        phi_close = 90.0 - th  # 0=open, 90=closed
        if abs(beta) < 1e-12:
            u = 1.0
        else:
            # u chosen so that damper_r = K/u^2 => K*exp(beta*phi/90)
            u = math.exp(-0.5 * beta * (phi_close / 90.0))
        u = _clamp(u, 1e-6, 1.0)

    else:
        raise ValueError(f"unknown model={model}")

    return _clamp(float(u), 1e-6, 1.0)


def u_to_angle(u: float, *, model: str = "sin", gamma: float = 1.0) -> float:
    """
    Inverse of angle_to_u.

    - "sin":    u = sin(theta)^gamma -> theta = asin(u^(1/gamma))
    - "linear": u = (theta/90)^gamma -> theta = 90*u^(1/gamma)

    - "exp":    (LEGACY) u = exp(-beta*(90-theta)/90)
               -> theta = 90 + 90*ln(u)/beta

    - "expk":   (RECOMMENDED) u = exp(-0.5*beta*(90-theta)/90)
               -> theta = 90 + 180*ln(u)/beta
    """
    uu = _clamp(float(u), 1e-6, 1.0)
    g = float(gamma)

    if model == "sin":
        base = uu ** (1.0 / g) if abs(g) > 1e-12 else uu
        base = _clamp(base, 0.0, 1.0)
        th = math.degrees(math.asin(base))

    elif model == "linear":
        base = uu ** (1.0 / g) if abs(g) > 1e-12 else uu
        th = 90.0 * base

    elif model == "exp":
        beta = g
        if abs(beta) < 1e-12:
            th = 90.0
        else:
            th = 90.0 + 90.0 * (math.log(uu) / beta)

    elif model == "expk":
        beta = g
        if abs(beta) < 1e-12:
            th = 90.0
        else:
            th = 90.0 + 180.0 * (math.log(uu) / beta)

    else:
        raise ValueError(f"unknown model={model}")

    return _clamp(float(th), 0.0, 90.0)


def with_damper_angle(
    net: Network,
    edge_id_to_theta: Dict[int, float],
    *,
    model: str = "sin",
    gamma: float = 1.0,
) -> Network:
    """
    Return a new Network with damper_u updated from damper angle(s).

    - edge_id_to_theta: edge_id -> theta(deg)
    - only edges with damper_k is not None will be updated
    """
    new_edges: List[Edge] = []
    for e in net.edges:
        if (e.id in edge_id_to_theta) and (e.damper_k is not None):
            th = float(edge_id_to_theta[e.id])
            u = angle_to_u(th, model=model, gamma=gamma)
            new_edges.append(replace(e, damper_u=u))
        else:
            new_edges.append(e)
    return Network(nodes=net.nodes, edges=new_edges, ref_node=net.ref_node)


# ----------------------------
# 1) 逐次（Gauss-Seidel）版
# ----------------------------
def balance_dampers_gauss_seidel(
    net: Network,
    *,
    targets_q: Dict[int, float],
    damper_edge_ids: Iterable[int],
    theta_center: float = 75.0,
    theta_band: float = 10.0,
    theta_init: Optional[Dict[int, float]] = None,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    outer_iters: int = 8,
    tol_q: float = 1e-3,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
) -> Tuple[Dict[int, float], SolveResult]:
    """
    Gauss-Seidel style balancing:
      - Iterate over dampers
      - For each damper, do bisection on theta within [center-band, center+band]
        to make edge flow close to target.

    注意:
      - 本関数は「局所的に調整」するため、強結合系では収束が遅い場合があります。
      - commissioning系の主アルゴリズムは control_cav.solve_cav_dampers_broyden 側です。
    """
    if fixed_p is None:
        fixed_p = {net.ref_node: 0.0}
    if d is None:
        d = {}

    theta_min = float(theta_center) - float(theta_band)
    theta_max = float(theta_center) + float(theta_band)

    thetas: Dict[int, float] = {int(eid): float(theta_center) for eid in damper_edge_ids}
    if theta_init:
        for eid, th in theta_init.items():
            if int(eid) in thetas:
                thetas[int(eid)] = float(th)

    last_res: SolveResult = solve_node_head(
        with_damper_angle(net, thetas, model=model, gamma=gamma),
        d=d,
        fixed_p=fixed_p,
        fan_q_init=fan_q_init,
        fan_q_cap=fan_q_cap,
    )

    def eval_q(eid: int, th: float) -> float:
        temp = dict(thetas)
        temp[int(eid)] = float(th)
        temp_net = with_damper_angle(net, temp, model=model, gamma=gamma)
        res = solve_node_head(
            temp_net,
            d=d,
            fixed_p=fixed_p,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )
        nonlocal last_res
        last_res = res
        return float(res.q[int(eid)])

    for _ in range(int(outer_iters)):
        all_ok = True
        for eid in list(damper_edge_ids):
            eid = int(eid)
            if eid not in targets_q:
                continue
            q_t = float(targets_q[eid])

            lo, hi = theta_min, theta_max
            q_lo = eval_q(eid, lo)
            q_hi = eval_q(eid, hi)

            # ensure q_lo <= q_hi
            if q_lo > q_hi:
                lo, hi = hi, lo
                q_lo, q_hi = q_hi, q_lo

            if q_t <= q_lo:
                thetas[eid] = lo
                q_now = q_lo
            elif q_t >= q_hi:
                thetas[eid] = hi
                q_now = q_hi
            else:
                for _k in range(30):
                    mid = 0.5 * (lo + hi)
                    q_mid = eval_q(eid, mid)
                    if q_mid < q_t:
                        lo = mid
                    else:
                        hi = mid
                    if abs(q_mid - q_t) < float(tol_q):
                        lo = hi = mid
                        break
                thetas[eid] = 0.5 * (lo + hi)
                q_now = eval_q(eid, thetas[eid])

            if abs(q_now - q_t) > float(tol_q):
                all_ok = False

        if all_ok and last_res.converged:
            break

    final_net = with_damper_angle(net, thetas, model=model, gamma=gamma)
    final_res = solve_node_head(
        final_net,
        d=d,
        fixed_p=fixed_p,
        fan_q_init=fan_q_init,
        fan_q_cap=fan_q_cap,
    )
    return thetas, final_res


# ----------------------------
# 2) “ベクトル更新” API（現状は安全にGSへフォールバック）
# ----------------------------
def balance_dampers_vector_update(
    net: Network,
    *,
    targets_q: Dict[int, float],
    damper_edge_ids: Iterable[int],
    theta_center: float = 75.0,
    theta_band: float = 10.0,
    theta_init: Optional[Dict[int, float]] = None,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    outer_iters: int = 10,
    tol_q: float = 1e-3,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
) -> Tuple[Dict[int, float], SolveResult]:
    """
    互換のための関数（以前のファイル構造に合わせる）。
    実運用の主役は control_cav.solve_cav_dampers_broyden なので、
    ここはまず “壊れないこと” を優先し、Gauss-Seidel に委譲します。
    """
    return balance_dampers_gauss_seidel(
        net,
        targets_q=targets_q,
        damper_edge_ids=damper_edge_ids,
        theta_center=theta_center,
        theta_band=theta_band,
        theta_init=theta_init,
        fixed_p=fixed_p,
        d=d,
        model=model,
        gamma=gamma,
        outer_iters=outer_iters,
        tol_q=tol_q,
        fan_q_init=fan_q_init,
        fan_q_cap=fan_q_cap,
    )


def balance_dampers_to_targets(
    net: Network,
    *,
    targets_q: Dict[int, float],
    damper_edge_ids: Iterable[int],
    method: str = "vector",
    theta_center: float = 75.0,
    theta_band: float = 10.0,
    theta_init: Optional[Dict[int, float]] = None,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    outer_iters: int = 10,
    tol_q: float = 1e-3,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
) -> Tuple[Dict[int, float], SolveResult]:
    """
    Public wrapper used by tests and some legacy callers.

    method:
      - "gauss_seidel"
      - "vector" (currently same as GS fallback)
    """
    m = str(method).lower().strip()
    if m in ("gauss", "gs", "gauss_seidel", "gauss-seidel"):
        return balance_dampers_gauss_seidel(
            net,
            targets_q=targets_q,
            damper_edge_ids=damper_edge_ids,
            theta_center=theta_center,
            theta_band=theta_band,
            theta_init=theta_init,
            fixed_p=fixed_p,
            d=d,
            model=model,
            gamma=gamma,
            outer_iters=outer_iters,
            tol_q=tol_q,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )
    elif m in ("vector", "vec", "vector_update"):
        return balance_dampers_vector_update(
            net,
            targets_q=targets_q,
            damper_edge_ids=damper_edge_ids,
            theta_center=theta_center,
            theta_band=theta_band,
            theta_init=theta_init,
            fixed_p=fixed_p,
            d=d,
            model=model,
            gamma=gamma,
            outer_iters=outer_iters,
            tol_q=tol_q,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )
    else:
        raise ValueError(f"unknown method={method}")
