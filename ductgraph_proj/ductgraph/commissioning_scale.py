from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Tuple, Iterable

from ductgraph.model import Network
from ductgraph.solver_nodehead import SolveResult
from ductgraph.control_damper import with_damper_angle
from ductgraph.control_speed import with_speed_ratio
from ductgraph.control_cav import solve_cav_dampers_broyden, CavSolveResult
from ductgraph.control_fullopen import eval_fullopen_flows


@dataclass
class ScalingCaseResult:
    name: str
    speed_ratio: float
    thetas: Dict[int, float]          # active cav edges only（offは含めない）
    res: SolveResult
    ok: bool
    msg: str
    max_err: float                    # undershoot deficits max (不足NG指標). ok のとき 0
    saturated_90: List[int]           # active edges where theta>=89.999 and still under
    achieved_q_abs: Dict[int, float]  # active edges abs(q)


@dataclass
class CommissionScaleResult:
    # full-load (band criterion)
    speed_full: float
    res_full: SolveResult

    # scaling cases
    cases: List[ScalingCaseResult]

    # selected index terminal used for full-load speed tuning
    index_edge_id: Optional[int] = None

    # strict commissioning status for full-load target opening band
    full_load_band_ok: bool = True
    full_load_index_theta: Optional[float] = None

    # full-load diagnostics
    full_load_diag: str = "ok"
    full_load_diag_msg: str = ""
    full_load_critical_edges: List[int] = field(default_factory=list)
    full_load_critical_thetas: List[float] = field(default_factory=list)


    @property
    def q65(self) -> float:
        # Deprecated: retained for backward compatibility only.
        return float("nan")

    @property
    def q85(self) -> float:
        # Deprecated: retained for backward compatibility only.
        return float("nan")


def _absq(res: SolveResult, edge_id: int) -> float:
    return float(abs(res.q[edge_id]))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _with_forced_damper_u(net: Network, edge_id_to_u: Dict[int, float]) -> Network:
    new_edges = []
    for e in net.edges:
        if (e.id in edge_id_to_u) and (e.damper_k is not None):
            u = _clamp(float(edge_id_to_u[e.id]), 1e-6, 1.0)
            new_edges.append(replace(e, damper_u=u))
        else:
            new_edges.append(e)
    return Network(nodes=net.nodes, edges=new_edges, ref_node=net.ref_node)


def speed_linear_scale(speed_full: float, qsum_active: float, qsum_full: float) -> float:
    """
    実務でよくある線形スケール：
      speed_ratio_case = speed_full * (Qsum_active / Qsum_full)

    ※ ただし物理的に「その風量が出ない」ケースはあり得る。
       その場合は “不足NG（全開でも不足）” を検出・レポートする。
    """
    if qsum_full <= 0:
        raise ValueError("qsum_full must be >0")
    return float(speed_full) * float(qsum_active) / float(qsum_full)


def solve_cav_at_speed(
    net: Network,
    *,
    speed_ratio: float,
    fan_edge_ids: List[int],
    active_cav_edge_ids: List[int],
    targets_q: Dict[int, float],  # active edges only
    # off terminals handling (partial operation)
    closed_cav_edge_ids: Optional[Iterable[int]] = None,
    theta_off: float = 0.0,
    off_damper_u: Optional[float] = None,
    # solver params
    theta0: float = 45.0,
    theta_min: float = 0.0,
    theta_max: float = 90.0,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    tol_q: float = 3e-3,
    max_iters: int = 80,
    alpha: float = 0.7,
    H0_gain: float = 1.0,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
) -> CavSolveResult:
    """
    speed_ratio を固定し、active な CAV 端末だけ theta(0..90) を同時に解く。

    部分運転では off 端末を “閉止” しておく必要がある（閉めないと流れてしまう）ため、
    closed_cav_edge_ids を与えた場合は、それらを theta_off で固定した状態のネットに対して解く。
    """
    if fixed_p is None:
        fixed_p = {net.ref_node: 0.0}
    if d is None:
        d = {}

    # speed 적용
    net_s = with_speed_ratio(net, speed_ratio, fan_edge_ids=fan_edge_ids)

    # close OFF terminals (important for partial operation)
    if closed_cav_edge_ids:
        closed_ids = [int(eid) for eid in closed_cav_edge_ids]
        if off_damper_u is None:
            close_dict = {eid: float(theta_off) for eid in closed_ids}
            net_s = with_damper_angle(net_s, close_dict, model=model, gamma=gamma)
        else:
            u_off = _clamp(float(off_damper_u), 1e-6, 1.0)
            net_s = _with_forced_damper_u(net_s, {eid: u_off for eid in closed_ids})

    out = solve_cav_dampers_broyden(
        net_s,
        cav_edge_ids=list(active_cav_edge_ids),
        targets_q=dict(targets_q),
        theta0=theta0,
        theta_min=theta_min,
        theta_max=theta_max,
        fixed_p=fixed_p,
        d=d,
        model=model,
        gamma=gamma,
        tol_q=tol_q,
        max_iters=max_iters,
        alpha=alpha,
        H0_gain=H0_gain,
        fan_q_init=fan_q_init,
        fan_q_cap=fan_q_cap,
    )
    return out


def commission_and_scale(
    net: Network,
    *,
    fan_edge_ids: List[int],
    full_active_cav_edge_ids: List[int],          # active CAV edges at full-load
    q_design_by_edge: Dict[int, float],           # design flow per terminal edge
    scaling_cases: List[Tuple[str, List[int]]],   # (case_name, active_edge_ids)
    seed_index_edge_id: Optional[int] = None,     # fallback index when full-load solve is infeasible
    theta_center: float = 75.0,
    theta_band: float = 10.0,
    theta_off_deg: float = 0.0,
    off_damper_u: Optional[float] = 1e-3,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    s_min: float = 0.2,
    s_max: float = 3.0,
    tol_q: float = 3e-3,
    eps_under_rel: float = 5e-3,
    cav_max_iters: int = 200,
    cav_alpha: float = 0.5,
    cav_H0_gain: float = 1.0,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
) -> CommissionScaleResult:
    """
    commissioning + scaling (CAV再現版)

    方針：
      A) 全台運転：全端末同時成立 + 厳しい端末群の開度帯で fan speed を決定
      B) 部分運転：
         - speed は設計風量比でスケール
         - CAV は「一致（match）」まで追い込む
         - 一致できない場合は WARN
         - 不足は NG
         - 全開でも不足は 能力不足 WARN
    """
    if fixed_p is None:
        fixed_p = {net.ref_node: 0.0}
    if d is None:
        d = {}

    # --- A) full-load speed tuning ---
    # Full-active solve first, then choose baseline speed from severe-terminal group.
    full_edges = [int(eid) for eid in full_active_cav_edge_ids if eid in q_design_by_edge]
    if not full_edges:
        raise ValueError("no full-load active edges in q_design_by_edge")

    full_targets = {eid: float(q_design_by_edge[eid]) for eid in full_edges}
    q_min_req_full = {eid: full_targets[eid] * (1.0 - eps_under_rel) for eid in full_edges}
    critical_n = max(1, min(2, len(full_edges)))
    band_lo = float(theta_center) - float(theta_band)
    band_hi = float(theta_center) + float(theta_band)

    def _solve_full_active_at_speed(speed_ratio: float) -> CavSolveResult:
        return solve_cav_at_speed(
            net,
            speed_ratio=float(speed_ratio),
            fan_edge_ids=fan_edge_ids,
            active_cav_edge_ids=list(full_edges),
            targets_q=full_targets,
            closed_cav_edge_ids=None,
            fixed_p=fixed_p,
            d=d,
            model=model,
            gamma=gamma,
            tol_q=tol_q,
            max_iters=cav_max_iters,
            alpha=cav_alpha,
            H0_gain=cav_H0_gain,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )

    n_grid = 25
    # sample tuple:
    # (speed, feasible, in_band(topN), top_pairs[(eid,theta)], cav_result)
    samples_scan: List[Tuple[float, bool, bool, List[Tuple[int, float]], CavSolveResult]] = []
    samples_ok: List[Tuple[float, bool, List[Tuple[int, float]], CavSolveResult]] = []
    sample_cache: Dict[float, Tuple[bool, bool, List[Tuple[int, float]], CavSolveResult]] = {}
    last_cav: Optional[CavSolveResult] = None
    any_capacity_shortage = False
    any_solver_unstable = False

    def _theta_pairs(cav: CavSolveResult) -> List[Tuple[int, float]]:
        if (not cav.res.converged) or (not cav.thetas):
            return []
        pairs: List[Tuple[int, float]] = []
        for eid in full_edges:
            if eid not in cav.thetas:
                return []
            th = float(cav.thetas[eid])
            if th != th:
                return []
            pairs.append((int(eid), th))
        return pairs

    def _full_feasible(cav: CavSolveResult, theta_pairs: List[Tuple[int, float]]) -> bool:
        if not theta_pairs:
            return False
        for eid in full_edges:
            if _absq(cav.res, eid) < q_min_req_full[eid]:
                return False
        return True

    def _eval_full_sample(speed_ratio: float) -> Tuple[bool, bool, List[Tuple[int, float]], CavSolveResult]:
        key = round(float(speed_ratio), 12)
        cached = sample_cache.get(key)
        if cached is not None:
            return cached

        cav = _solve_full_active_at_speed(float(speed_ratio))
        theta_pairs = _theta_pairs(cav)
        feasible = _full_feasible(cav, theta_pairs)
        top_pairs: List[Tuple[int, float]] = []
        in_band = False

        if feasible:
            theta_pairs.sort(key=lambda x: x[1], reverse=True)
            top_pairs = theta_pairs[:critical_n]
            top_thetas = [th for _, th in top_pairs]
            in_band = all((band_lo <= th <= band_hi) for th in top_thetas)

        out = (bool(feasible), bool(in_band), top_pairs, cav)
        sample_cache[key] = out
        return out

    for i in range(n_grid + 1):
        s_i = float(s_min) + (float(s_max) - float(s_min)) * (float(i) / float(n_grid))
        feasible_i, in_band_i, top_pairs_i, cav_i = _eval_full_sample(s_i)
        last_cav = cav_i
        samples_scan.append((float(s_i), bool(feasible_i), bool(in_band_i), list(top_pairs_i), cav_i))

        if feasible_i:
            samples_ok.append((float(s_i), bool(in_band_i), list(top_pairs_i), cav_i))
            continue

        # non-feasible sample: classify shortage vs unstable
        try:
            q_fo = eval_fullopen_flows(
                net,
                speed_ratio=float(s_i),
                fan_edge_ids=fan_edge_ids,
                cav_edge_ids=full_edges,
                fixed_p=fixed_p,
                d=d,
                model=model,
                gamma=gamma,
                fan_q_init=fan_q_init,
                fan_q_cap=fan_q_cap,
            )
            unders_fo = [eid for eid in full_edges if float(q_fo[eid]) < q_min_req_full[eid]]
            if unders_fo:
                any_capacity_shortage = True
            else:
                any_solver_unstable = True
        except RuntimeError:
            any_solver_unstable = True

    full_load_diag = "ok"
    full_load_diag_msg = ""
    full_load_critical_edges: List[int] = []
    full_load_critical_thetas: List[float] = []

    if samples_ok:
        def _dist_to_band(theta: float) -> float:
            if theta < band_lo:
                return float(band_lo - theta)
            if theta > band_hi:
                return float(theta - band_hi)
            return 0.0

        def _score_no_inband(sample: Tuple[float, bool, List[Tuple[int, float]], CavSolveResult]) -> Tuple[float, float, float, float]:
            s_v, _in_band_v, top_pairs_v, _cav_v = sample
            top_thetas_v = [float(th) for _, th in top_pairs_v]
            max_theta_v = max(top_thetas_v)
            over_open = max(0.0, max_theta_v - band_hi)  # prioritize avoiding 90deg saturation
            over_closed = max(0.0, band_lo - max_theta_v)
            dist_sum = sum(_dist_to_band(th) for th in top_thetas_v)
            return (float(over_open), float(over_closed), float(dist_sum), float(s_v))

        # Dense rescan so band candidates are not missed by coarse grid.
        n_dense = 120
        dense_ok: List[Tuple[float, bool, List[Tuple[int, float]], CavSolveResult]] = []
        for i in range(n_dense + 1):
            s_i = float(s_min) + (float(s_max) - float(s_min)) * (float(i) / float(n_dense))
            feasible_i, in_band_i, top_pairs_i, cav_i = _eval_full_sample(s_i)
            if feasible_i:
                dense_ok.append((float(s_i), bool(in_band_i), list(top_pairs_i), cav_i))

        in_band_samples = [s for s in dense_ok if s[1]]
        if in_band_samples:
            # Prefer minimum required speed among in-band candidates.
            chosen = min(in_band_samples, key=lambda x: float(x[0]))

            # Refine the left boundary for true minimum in-band speed.
            seed_s = float(chosen[0])
            lower_candidates = [
                float(s)
                for s, feasible, in_band, _top_pairs, _cav in samples_scan
                if float(s) < seed_s and (not (feasible and in_band))
            ]
            if lower_candidates:
                lo = max(lower_candidates)
                hi = seed_s
                best_refined = chosen

                for _ in range(28):
                    mid = 0.5 * (lo + hi)
                    feasible_m, in_band_m, top_pairs_m, cav_m = _eval_full_sample(mid)
                    if feasible_m and in_band_m:
                        hi = mid
                        best_refined = (float(mid), bool(in_band_m), list(top_pairs_m), cav_m)
                    else:
                        lo = mid
                chosen = best_refined

            full_load_diag = "ok"
            full_load_diag_msg = ""
        else:
            # If no in-band candidate exists, select the best-fit feasible point.
            # Priority: avoid 90deg saturation, then reduce band distance, then lower speed.
            chosen = min(dense_ok, key=_score_no_inband)

        best_s = float(chosen[0])
        top_pairs = list(chosen[2])
        cav_best = chosen[3]
        res_band = cav_best.res
        _index_edge = int(top_pairs[0][0])
        _theta_idx = float(top_pairs[0][1])
        full_load_critical_edges = [int(eid) for eid, _ in top_pairs]
        full_load_critical_thetas = [float(th) for _, th in top_pairs]

        if not bool(chosen[1]):
            top_thetas = [th for _, th in top_pairs]
            if max(top_thetas) < band_lo:
                full_load_diag = "over_static"
                full_load_diag_msg = "severe terminals are too closed (high static side)"
            elif min(top_thetas) > band_hi:
                full_load_diag = "under_static"
                full_load_diag_msg = "severe terminals are too open (low static side)"
            else:
                m = sum(top_thetas) / float(len(top_thetas))
                full_load_diag = "over_static" if m < float(theta_center) else "under_static"
                full_load_diag_msg = "severe terminals do not fit target band"

    else:
        assert last_cav is not None
        best_s = float(s_max)
        res_band = last_cav.res
        seed_edge = seed_index_edge_id
        _index_edge = int(seed_edge if (seed_edge in full_edges) else full_edges[0])
        _theta_idx = float("nan")
        if any_capacity_shortage:
            full_load_diag = "capacity_shortage"
            full_load_diag_msg = "design flow cannot be met even at full-open on scanned speeds"
        else:
            full_load_diag = "solver_unstable"
            full_load_diag_msg = "full-load CAV simultaneous solve is unstable/non-convergent"

    full_load_band_ok = (
        bool(full_load_critical_thetas)
        and all((band_lo <= float(th) <= band_hi) for th in full_load_critical_thetas)
    )

    if not res_band.converged:
        return CommissionScaleResult(
            float(best_s),
            res_band,
            cases=[],
            index_edge_id=int(_index_edge),
            full_load_band_ok=bool(full_load_band_ok),
            full_load_index_theta=float(_theta_idx) if (_theta_idx == _theta_idx) else None,
            full_load_diag=str(full_load_diag),
            full_load_diag_msg=str(full_load_diag_msg),
            full_load_critical_edges=list(full_load_critical_edges),
            full_load_critical_thetas=list(full_load_critical_thetas),
        )

    qsum_full = sum(float(q_design_by_edge[eid]) for eid in full_active_cav_edge_ids)
    cases_out: List[ScalingCaseResult] = []

    # --- B) scaling cases ---
    for name, active_edges in scaling_cases:
        active_edges = list(active_edges)
        qsum_active = sum(float(q_design_by_edge[eid]) for eid in active_edges)
        s_case = speed_linear_scale(best_s, qsum_active=qsum_active, qsum_full=qsum_full)
        s_case_raw = float(s_case)
        s_case = min(max(float(s_case), float(s_min)), float(s_max))
        s_case_clamped = (abs(float(s_case) - s_case_raw) > 1e-12)

        targets = {eid: float(q_design_by_edge[eid]) for eid in active_edges}
        q_min_req = {eid: targets[eid] * (1.0 - eps_under_rel) for eid in active_edges}

        off_edges = [eid for eid in full_active_cav_edge_ids if eid not in set(active_edges)]

        # --- (1) CAV control


        # --- (1) CAV control (matchまで追い込む) ---
        cav = solve_cav_at_speed(
            net,
            speed_ratio=s_case,
            fan_edge_ids=fan_edge_ids,
            active_cav_edge_ids=active_edges,
            targets_q=targets,
            closed_cav_edge_ids=off_edges,
            theta_off=float(theta_off_deg),
            off_damper_u=off_damper_u,
            fixed_p=fixed_p,
            d=d,
            model=model,
            gamma=gamma,
            tol_q=tol_q,
            max_iters=cav_max_iters,
            alpha=cav_alpha,
            H0_gain=cav_H0_gain,
            fan_q_init=fan_q_init,
            fan_q_cap=fan_q_cap,
        )

        res = cav.res
        thetas_active = dict(cav.thetas) if cav.thetas else {}

        achieved: Dict[int, float] = {}
        unders: List[Tuple[int, float]] = []
        overs: List[Tuple[int, float]] = []

        if res.converged:
            achieved = {eid: _absq(res, eid) for eid in active_edges}
            for eid in active_edges:
                q = achieved[eid]
                if q < q_min_req[eid]:
                    unders.append((eid, q))
                if q > (targets[eid] + tol_q):
                    overs.append((eid, q))

        # --- (2) 能力チェック（全開） ---
        fullopen_msgs: List[str] = []
        fullopen_unders: List[Tuple[int, float]] = []

        if (not res.converged) or unders:
            try:
                q_fo = eval_fullopen_flows(
                    net,
                    speed_ratio=s_case,
                    fan_edge_ids=fan_edge_ids,
                    cav_edge_ids=active_edges,
                    fixed_p=fixed_p,
                    d=d,
                    model=model,
                    gamma=gamma,
                    fan_q_init=fan_q_init,
                    fan_q_cap=fan_q_cap,
                )
                for eid in active_edges:
                    q = float(q_fo[eid])
                    if q < q_min_req[eid]:
                        fullopen_unders.append((eid, q))
                        fullopen_msgs.append(
                            f"edge {eid}: WARN full-open undershoot: Q={q:.6g} < Qdes={targets[eid]:.6g}"
                        )
            except RuntimeError:
                fullopen_msgs.append("WARN full-open check did not converge")

        # --- 判定 ---
        ok = True
        msgs: List[str] = []

        if not res.converged:
            ok = False
            msgs.append(cav.msg if cav.msg else "inner solve did not converge")

        if unders:
            ok = False
            msgs.append(
                "FAIL undershoot (scaling): " +
                ", ".join([f"{eid}:{q:.6g}<{q_min_req[eid]:.6g}" for eid, q in unders])
            )

        if res.converged and (not cav.ok):
            msgs.append(cav.msg if cav.msg else "WARN not matched (CAV)")

        if fullopen_msgs:
            msgs.extend(fullopen_msgs)

        if s_case_clamped:
            msgs.append(f"WARN speed clamped: {s_case_raw:.6g}->{s_case:.6g}")

        if overs:
            msgs.append(
                "WARN overshoot: " +
                ", ".join([f"{eid}:{q:.6g}>{targets[eid] + tol_q:.6g}" for eid, q in overs])
            )
        msg = "; ".join([m for m in msgs if m]) if msgs else "ok"

        if unders:
            deficits = [(q_min_req[eid] - q) for eid, q in unders]
            max_err = float(max(deficits))
        elif not res.converged:
            max_err = float("inf")
        else:
            max_err = 0.0

        saturated_90 = [eid for eid, _ in fullopen_unders]

        cases_out.append(
            ScalingCaseResult(
                name=name,
                speed_ratio=float(s_case),
                thetas=thetas_active,
                res=res,
                ok=ok,
                msg=msg,
                max_err=max_err,
                saturated_90=saturated_90,
                achieved_q_abs=achieved,
            )
        )

    return CommissionScaleResult(
        float(best_s),
        res_band,
        cases=cases_out,
        index_edge_id=int(_index_edge),
        full_load_band_ok=bool(full_load_band_ok),
        full_load_index_theta=float(_theta_idx) if (_theta_idx == _theta_idx) else None,
        full_load_diag=str(full_load_diag),
        full_load_diag_msg=str(full_load_diag_msg),
        full_load_critical_edges=list(full_load_critical_edges),
        full_load_critical_thetas=list(full_load_critical_thetas),
    )
