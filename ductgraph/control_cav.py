from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np

from ductgraph.model import Network
from ductgraph.solver_nodehead import solve_node_head, SolveResult
from ductgraph.control_damper import with_damper_angle


@dataclass
class CavSolveResult:
    thetas: Dict[int, float]     # edge_id -> theta(deg)
    res: SolveResult             # 最後の network solve 結果
    ok: bool                     # match達成フラグ: res.converged かつ max_i | |Q_i| - Qdes_i | <= tol_q
    msg: str                     # "ok" or "WARN ..." or failure reason
    iters: int
    max_err: float               # |Q|-Qtarget の最大絶対誤差（診断用・合否ではない）


def _absq(res: SolveResult, edge_id: int) -> float:
    return float(abs(res.q[edge_id]))


def _eval(
    net: Network,
    thetas: Dict[int, float],
    *,
    fixed_p: Dict[int, float],
    d: Dict[int, float],
    model: str,
    gamma: float,
    fan_q_init: float,
    fan_q_cap: float,
) -> SolveResult:
    net_t = with_damper_angle(net, thetas, model=model, gamma=gamma)
    return solve_node_head(net_t, d=d, fixed_p=fixed_p, fan_q_init=fan_q_init, fan_q_cap=fan_q_cap)


def solve_cav_dampers_broyden(
    net: Network,
    *,
    cav_edge_ids: List[int],
    targets_q: Dict[int, float],  # edge_id -> q_design (正値)
    theta0: float = 45.0,
    theta_min: float = 0.0,
    theta_max: float = 90.0,
    fixed_p: Optional[Dict[int, float]] = None,
    d: Optional[Dict[int, float]] = None,
    model: str = "sin",
    gamma: float = 1.0,
    tol_q: float = 3e-3,          # NOW: match tolerance (absolute) for |Q|-Qdes
    max_iters: int = 60,
    alpha: float = 0.7,
    H0_gain: float = 1.0,
    fan_q_init: float = 2.0,
    fan_q_cap: float = 80.0,
    # numeric absorption (undershoot only) for reporting
    eps_under_rel: float = 5e-3,
) -> CavSolveResult:
    """
    CAV端末のダンパ角度（theta）を同時更新（Broyden準Newton）で解く。

    制御仕様（あなたの要求に合わせて明確化）：
      - 目標は「一致」：max_i | |Q_i| - Qdes_i | <= tol_q まで追い込む
      - それが出来ない場合（未収束 or 反復上限）でも、最後の状態を返し、
        呼び出し側（commissioning_scale）が
           * 不足NG
           * θ=90でも不足（能力不足警告）
        を判定する。

    返り値 ok の意味：
      - ok=True  : res.converged かつ 一致達成（上の tol_q 条件）
      - ok=False : それ以外
    """
    if fixed_p is None:
        fixed_p = {net.ref_node: 0.0}
    if d is None:
        d = {}

    eids = list(cav_edge_ids)
    K = len(eids)

    if K == 0:
        res0 = _eval(
            net, {},
            fixed_p=fixed_p, d=d, model=model, gamma=gamma,
            fan_q_init=fan_q_init, fan_q_cap=fan_q_cap,
        )
        ok0 = bool(res0.converged)
        msg0 = "ok" if ok0 else "inner solve did not converge"
        return CavSolveResult({}, res0, ok0, msg0, 0, 0.0)

    import numpy as np

    # initial theta
    theta = np.full((K,), float(theta0), dtype=float)
    theta = np.clip(theta, theta_min, theta_max)

    def theta_dict(th_vec: np.ndarray) -> Dict[int, float]:
        return {eid: float(th_vec[i]) for i, eid in enumerate(eids)}

    def eval_qabs(res: SolveResult) -> np.ndarray:
        return np.array([_absq(res, eid) for eid in eids], dtype=float)

    qtar_vec = np.array([float(targets_q[eid]) for eid in eids], dtype=float)
    q_min_req = qtar_vec * (1.0 - float(eps_under_rel))

    def eval_err(res: SolveResult) -> tuple[float, list[str]]:
        """Return (max_abs_err, detail_msgs). Assumes res.converged."""
        qabs = eval_qabs(res)
        diff = qabs - qtar_vec
        max_abs = float(np.max(np.abs(diff)))
        msgs: list[str] = []
        # also build unders/overs diagnostics
        for i, eid in enumerate(eids):
            if qabs[i] < q_min_req[i]:
                msgs.append(f"{eid}:UNDER {qabs[i]:.6g}<{q_min_req[i]:.6g}")
            if qabs[i] > (qtar_vec[i] + float(tol_q)):
                msgs.append(f"{eid}:OVER {qabs[i]:.6g}>{(qtar_vec[i]+float(tol_q)):.6g}")
        return max_abs, msgs

    def F_of(th_vec: np.ndarray) -> tuple[np.ndarray, SolveResult]:
        res = _eval(
            net,
            theta_dict(th_vec),
            fixed_p=fixed_p, d=d, model=model, gamma=gamma,
            fan_q_init=fan_q_init, fan_q_cap=fan_q_cap,
        )
        if not res.converged:
            return np.full((K,), 1e9, dtype=float), res
        qabs = eval_qabs(res)
        return (qabs - qtar_vec), res

    # initial inverse-Jacobian approximation
    H = np.eye(K, dtype=float) * float(H0_gain)

    F, res = F_of(theta)
    if not res.converged:
        return CavSolveResult(theta_dict(theta), res, False, "inner solve did not converge", 0, float("inf"))

    max_abs_err, diag = eval_err(res)
    if max_abs_err <= float(tol_q):
        return CavSolveResult(theta_dict(theta), res, True, "ok", 0, max_abs_err)

    last_F = F
    last_theta = theta.copy()

    for it in range(1, max_iters + 1):
        # dtheta = -H F
        dtheta = -H.dot(last_F)

        theta_trial = last_theta + float(alpha) * dtheta
        theta_trial = np.clip(theta_trial, theta_min, theta_max)

        F_new, res_new = F_of(theta_trial)

        # if inner not converged: reduce step
        if not res_new.converged:
            alpha2 = float(alpha) * 0.5
            theta_trial2 = np.clip(last_theta + alpha2 * dtheta, theta_min, theta_max)
            F_new2, res_new2 = F_of(theta_trial2)
            if not res_new2.converged:
                return CavSolveResult(theta_dict(last_theta), res_new2, False, "inner solve did not converge", it, float("inf"))
            theta_trial, F_new, res_new = theta_trial2, F_new2, res_new2

        # match check
        max_abs_err, diag = eval_err(res_new)
        if max_abs_err <= float(tol_q):
            return CavSolveResult(theta_dict(theta_trial), res_new, True, "ok", it, max_abs_err)

        # Broyden update: H_{k+1} = H_k + ((s - H y) s^T H) / (s^T H y)
        s_vec = (theta_trial - last_theta).reshape((K, 1))
        y_vec = (F_new - last_F).reshape((K, 1))

        Hy = H.dot(y_vec)
        denom = float((s_vec.T.dot(Hy))[0, 0])
        if abs(denom) > 1e-12:
            H = H + ((s_vec - Hy).dot(s_vec.T).dot(H)) / denom

        last_theta = theta_trial
        last_F = F_new

    # not matched within max_iters: return last state with ok=False, and include diag
    max_abs_err, diag = eval_err(res_new) if res_new.converged else (float("inf"), [])
    msg = "WARN not matched within max_iters"
    if diag:
        msg += "; " + ", ".join(diag)
    return CavSolveResult(theta_dict(last_theta), res_new, False, msg, max_iters, max_abs_err)

