from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .model import Network
from .solver_nodehead import solve_node_head, SolveResult
from .control_damper import with_damper_angle
from .control_speed import with_speed_ratio, tune_speed_for_maxload


@dataclass
class CommissionResult:
    """
    speed_ratio : fan speed ratio applied to fan edges (dimensionless)
    thetas      : damper angles [deg] for edges (only those you tuned / fixed)
    res         : final SolveResult
    ok          : pass/fail
    msg         : message (warnings etc.)
    """
    speed_ratio: float
    thetas: Dict[int, float]
    res: SolveResult
    ok: bool
    msg: str = ""


def _absq(res: SolveResult, edge_id: int) -> float:
    """Absolute flow magnitude on an edge (m3/s)."""
    return abs(res.q[edge_id])


def _eval_network(
    net: Network,
    *,
    thetas: Dict[int, float],
    model: str,
    gamma: float,
    d: Dict[int, float],
    fixed_p: Dict[int, float],
    fan_q_init: float,
    fan_q_cap: float,
) -> SolveResult:
    """
    Evaluate network with damper angles applied.

    thetas  : {edge_id: theta_deg}
    model   : damper model name (e.g. "sin")
    gamma   : damper model parameter
    fixed_p : node pressure BCs (Pa)
    d       : extra per-edge params (existing code convention)
    """
    net2 = with_damper_angle(net, thetas, model=model, gamma=gamma)
    return solve_node_head(net2, fixed_p=fixed_p, d=d, fan_q_init=fan_q_init, fan_q_cap=fan_q_cap)


def _bisection_theta_for_atleast(
    net: Network,
    edge_id: int,
    q_design: float,
    *,
    theta_lo: float,
    theta_hi: float,
    thetas_fixed: Dict[int, float],
    model: str,
    gamma: float,
    d: Dict[int, float],
    fixed_p: Dict[int, float],
    fan_q_init: float,
    fan_q_cap: float,
    # --- semantics ---
    eps_guard_rel: float,   # tuning target bias: aim slightly ABOVE design
    tol_conv_rel: float,    # convergence stop (NOT pass/fail)
    max_iter: int = 35,
) -> Tuple[float, SolveResult, str]:
    """
    Find the MINIMUM theta in [theta_lo, theta_hi] such that:
        Q(edge_id) >= q_design * (1 + eps_guard_rel)

    This "eps_guard_rel" is NOT an acceptance tolerance.
    It's a tuning bias to avoid tiny numeric undershoots in final solution.

    Early stop when:
        Q <= q_design * (1 + tol_conv_rel)
    """
    q_target = q_design * (1.0 + eps_guard_rel)
    q_stop_hi = q_design * (1.0 + tol_conv_rel)

    res_lo = _eval_network(
        net,
        thetas={**thetas_fixed, edge_id: theta_lo},
        model=model, gamma=gamma, d=d, fixed_p=fixed_p,
        fan_q_init=fan_q_init, fan_q_cap=fan_q_cap,
    )
    res_hi = _eval_network(
        net,
        thetas={**thetas_fixed, edge_id: theta_hi},
        model=model, gamma=gamma, d=d, fixed_p=fixed_p,
        fan_q_init=fan_q_init, fan_q_cap=fan_q_cap,
    )
    if (not res_lo.converged) or (not res_hi.converged):
        return theta_hi, res_hi, "network solve did not converge (theta bisection)"

    q_lo = _absq(res_lo, edge_id)
    q_hi = _absq(res_hi, edge_id)

    if q_hi < q_target:
        return theta_hi, res_hi, (
            f"INFEASIBLE: even theta_hi={theta_hi:.3g} gives Q={q_hi:.6g} < Qtarget={q_target:.6g}"
        )

    if q_lo >= q_target:
        return theta_lo, res_lo, ""

    lo, hi = theta_lo, theta_hi
    best_theta = theta_hi
    best_res = res_hi

    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        res_mid = _eval_network(
            net,
            thetas={**thetas_fixed, edge_id: mid},
            model=model, gamma=gamma, d=d, fixed_p=fixed_p,
            fan_q_init=fan_q_init, fan_q_cap=fan_q_cap,
        )
        if not res_mid.converged:
            lo = mid
            continue

        q_mid = _absq(res_mid, edge_id)

        if q_mid >= q_target:
            hi = mid
            best_theta, best_res = mid, res_mid
            if q_mid <= q_stop_hi:
                break
        else:
            lo = mid

    return best_theta, best_res, ""


def commission_two_stage(
    net: Network,
    *,
    fan_edge_ids: List[int],
    maxload_edge_id: int,
    other_damper_edge_ids: List[int],
    q_design: float,
    theta_center: float,
    theta_band: float,
    s_min: float,
    s_max: float,
    other_theta_min: float,
    other_theta_max: float,
    rounds: int,
    tol_q: float,  # legacy input: WARN threshold (absolute). DO NOT use as pass/fail.
    fixed_p: Dict[int, float],
    d: Dict[int, float],
    model: str = "sin",
    gamma: float = 1.0,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
    # --- split semantics ---
    eps_under_rel: float = 1e-4,  # final pass/fail numeric absorption (undershoot only)
    tol_conv_rel: float = 3e-3,   # convergence stop (NOT pass/fail)
    eps_guard_rel: float = 5e-4,  # speed tuning guard (aim slightly above design)
) -> CommissionResult:
    """
    Two-stage commissioning.

    Words + variables:
      - 設計風量 q_design [m3/s] : 端末が満たすべき基準流量（この関数では全端末同一を想定）
      - 端末流量 Q [m3/s]       : SolveResult.q[edge_id]（符号付きなので |Q| = _absq を使用）
      - ダンパ開度 theta [deg]  : 大きいほど開く（=流量が増える前提）

    Tolerances (用途分離):
      - eps_under_rel : 合否（不足NG）の数値誤差吸収（相対・下側のみ）
            pass iff  |Q_i| >= q_design * (1 - eps_under_rel)  for all terminals
      - tol_conv_rel  : 収束（止め時）の目安（相対・合否ではない）
      - eps_guard_rel : 速度チューニング時の上乗せ（相対・合否ではない）
            speed is tuned so that maxload at theta_center hits >= q_design*(1+eps_guard_rel)
      - tol_q         : 警告（authority band等）の目安（absolute・合否ではない）
    """
    # legacy warn threshold (absolute)
    tol_warn_rel = (tol_q / q_design) if (q_design > 0) else 0.0
    tol_warn_abs = tol_warn_rel * q_design

    # initial damper state: maxload fixed, others start fully open within throttle-only range
    thetas: Dict[int, float] = {maxload_edge_id: theta_center}
    for eid in other_damper_edge_ids:
        thetas[eid] = other_theta_max

    last_res: Optional[SolveResult] = None
    last_msg: str = ""
    best_s: float = 1.0

    q_min_req = q_design * (1.0 - eps_under_rel)            # pass/fail minimum
    q_target_speed = q_design * (1.0 + eps_guard_rel)       # target used only for tuning speed
    q_stop_hi = q_design * (1.0 + tol_conv_rel)             # convergence (not-too-over)

    def _q_maxload_at_speed(sratio: float) -> Tuple[float, SolveResult]:
        net_s = with_speed_ratio(net, sratio, fan_edge_ids=fan_edge_ids)
        res = _eval_network(
            net_s,
            thetas=thetas,
            model=model,
            gamma=gamma,
            d=d,
            fixed_p=fixed_p,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )
        q = _absq(res, maxload_edge_id)
        return q, res

    def _tune_speed_min_ge_target() -> Tuple[float, SolveResult, str]:
        # Need: maxload at theta_center is >= q_target_speed (guarded) (and therefore >= q_min_req)
        q_lo, res_lo = _q_maxload_at_speed(s_min)
        if not res_lo.converged:
            return s_min, res_lo, "network solve did not converge (speed tune @ s_min)"
        if q_lo >= q_target_speed:
            return s_min, res_lo, ""

        q_hi, res_hi = _q_maxload_at_speed(s_max)
        if not res_hi.converged:
            return s_max, res_hi, "network solve did not converge (speed tune @ s_max)"
        if q_hi < q_min_req:
            # Even acceptance minimum cannot be met at max speed => fan selection / bounds are wrong => FAIL
            return s_max, res_hi, (
                f"FAIL: maxload cannot reach design at theta_center even at s_max "
                f"(Q={q_hi:.6g} < {q_min_req:.6g})"
            )

        # bisection for minimal speed achieving q_target_speed
        lo, hi = s_min, s_max
        best = s_max
        best_res = res_hi
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            q_mid, res_mid = _q_maxload_at_speed(mid)
            if not res_mid.converged:
                # treat as too low (push upward)
                lo = mid
                continue
            if q_mid >= q_target_speed:
                best = mid
                best_res = res_mid
                hi = mid
            else:
                lo = mid
            if abs(hi - lo) <= 1e-6:
                break
        return best, best_res, ""

    for _round in range(1, rounds + 1):
        # --- Stage 1: tune speed so that maxload at theta_center is NOT under design (guarded) ---
        best_s, res_speed, msg_speed = _tune_speed_min_ge_target()
        if msg_speed.startswith("FAIL:"):
            return CommissionResult(best_s, dict(thetas), res_speed, False, msg_speed)
        if not res_speed.converged:
            return CommissionResult(best_s, dict(thetas), res_speed, False, msg_speed)

        net_s = with_speed_ratio(net, best_s, fan_edge_ids=fan_edge_ids)

        # --- Stage 2: tune other dampers (Gauss-Seidel) to achieve at-least design, then reduce overshoot ---
        infeasible_msgs: List[str] = []
        for eid in other_damper_edge_ids:
            fixed_thetas = {k: v for k, v in thetas.items() if k != eid}
            theta, res_e, msg = _bisection_theta_for_atleast(
                net_s,
                eid,
                q_design,
                theta_lo=other_theta_min,
                theta_hi=other_theta_max,
                thetas_fixed=fixed_thetas,
                model=model,
                gamma=gamma,
                d=d,
                fixed_p=fixed_p,
                fan_q_init=fan_q_init,
                fan_q_cap=fan_q_cap,
                eps_guard_rel=eps_guard_rel,
                tol_conv_rel=tol_conv_rel,
            )
            thetas[eid] = theta
            if msg:
                infeasible_msgs.append(f"edge {eid}: {msg}")

        # final solve this round
        res_all = _eval_network(
            net_s,
            thetas=thetas,
            model=model,
            gamma=gamma,
            d=d,
            fixed_p=fixed_p,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )
        last_res = res_all
        if not res_all.converged:
            return CommissionResult(best_s, dict(thetas), res_all, False, "network solve did not converge (final)")

        # --- PASS/FAIL (undershoot NG; numeric absorption only) ---
        unders: List[Tuple[int, float]] = []
        for eid in [maxload_edge_id, *other_damper_edge_ids]:
            q = _absq(res_all, eid)
            if q < q_min_req:
                unders.append((eid, q))

        if unders:
            last_msg = "WARN undershoot (will retry): " + ", ".join([f"{eid}:{q:.6g}" for eid, q in unders])
            if _round < rounds:
                continue
            return CommissionResult(best_s, dict(thetas), res_all, False, last_msg)

        # --- WARNINGS (not pass/fail) ---
        warn_msgs: List[str] = []
        warn_msgs.extend(infeasible_msgs)

        # maxload authority band check (warning only)
        th_min = theta_center - theta_band
        th_max = theta_center + theta_band
        thetas_min = dict(thetas); thetas_min[maxload_edge_id] = th_min
        thetas_max = dict(thetas); thetas_max[maxload_edge_id] = th_max
        res_min = _eval_network(
            net_s, thetas=thetas_min, model=model, gamma=gamma, d=d, fixed_p=fixed_p,
            fan_q_init=fan_q_init, fan_q_cap=fan_q_cap
        )
        res_max = _eval_network(
            net_s, thetas=thetas_max, model=model, gamma=gamma, d=d, fixed_p=fixed_p,
            fan_q_init=fan_q_init, fan_q_cap=fan_q_cap
        )
        if res_min.converged and res_max.converged:
            q_m_min = _absq(res_min, maxload_edge_id)
            q_m_max = _absq(res_max, maxload_edge_id)
            lo, hi = (q_m_min, q_m_max) if q_m_min <= q_m_max else (q_m_max, q_m_min)
            if not ((lo - tol_warn_abs) <= q_design <= (hi + tol_warn_abs)):
                warn_msgs.append(
                    f"WARN maxload authority: band[{th_min:.0f}-{th_max:.0f}] gives Q in [{lo:.6g},{hi:.6g}] target={q_design:.6g}"
                )

        # convergence stop (not pass/fail): others should not be too over
        ok_conv = True
        for eid in other_damper_edge_ids:
            if _absq(res_all, eid) > q_stop_hi:
                ok_conv = False
                break

        if ok_conv:
            msg = "; ".join(warn_msgs) if warn_msgs else "ok"
            return CommissionResult(best_s, dict(thetas), res_all, True, msg)

        # not converged yet -> iterate more
        last_msg = "; ".join(warn_msgs) if warn_msgs else ""

    msg = last_msg or "did not reach convergence within rounds"
    return CommissionResult(best_s, dict(thetas), last_res, True, ("WARN " + msg if msg else "ok"))
