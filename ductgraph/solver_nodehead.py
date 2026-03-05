import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional, List
from .model import Network


@dataclass
class SolveResult:
    p: Dict[int, float]
    q: Dict[int, float]
    converged: bool
    iters: int
    residual_norm: float


def _fan_poly(Q, a3, a2, a1, a0):
    return ((a3 * Q + a2) * Q + a1) * Q + a0


def _fan_poly_deriv(Q, a3, a2, a1):
    return (3.0 * a3 * Q + 2.0 * a2) * Q + a1


def _terminal_r(terminal_orifice):
    rho, cd, area = terminal_orifice
    ca = cd * area
    if ca <= 0:
        raise ValueError("terminal_orifice: Cd*Area must be > 0")
    return float(rho) / (2.0 * (ca ** 2))


def _damper_r(damper_k: Optional[float], damper_u: float) -> float:
    if damper_k is None:
        return 0.0
    u = float(damper_u)
    # avoid division by zero / negative
    u = max(u, 1e-6)
    return float(damper_k) / (u * u)


def solve_node_head(
    network: Network,
    d: Dict[int, float],
    *,
    fixed_p: Optional[Dict[int, float]] = None,
    max_iter: int = 80,
    tol: float = 1e-9,
    damping: float = 1.0,
    fan_q_init: float = 1.0,
    fan_q_cap: float = 10.0,
) -> SolveResult:

    if fixed_p is None:
        fixed_p = {network.ref_node: 0.0}
    else:
        fixed_p = dict(fixed_p)
        fixed_p.setdefault(network.ref_node, 0.0)

    nodes = network.nodes
    edges = network.edges

    node_ids = [n.id for n in nodes]
    node_index = {nid: i for i, nid in enumerate(node_ids)}
    n = len(nodes)
    m = len(edges)

    # incidence matrix A (n x m)
    A = np.zeros((n, m))
    for j, e in enumerate(edges):
        A[node_index[e.frm], j] = -1.0
        A[node_index[e.to], j] = +1.0

    # free pressure nodes
    free_nodes = [nid for nid in node_ids if nid not in fixed_p]
    free_index = {nid: i for i, nid in enumerate(free_nodes)}
    n_free = len(free_nodes)

    # fan edges (unknown Q)
    fan_edges: List[int] = [j for j, e in enumerate(edges) if e.fan_poly is not None]
    k_fan = len(fan_edges)
    fan_pos = {ej: i for i, ej in enumerate(fan_edges)}  # edge index -> position

    # unknown vector x = [p_free (n_free), Q_fan (k_fan)]
    x = np.zeros(n_free + k_fan)

    # init Q_fan
    for ej, pos in fan_pos.items():
        x[n_free + pos] = float(fan_q_init)

    def build_p_all(p_free_vec):
        p = np.zeros(n)
        for nid, pv in fixed_p.items():
            p[node_index[nid]] = float(pv)
        for nid, i in free_index.items():
            p[node_index[nid]] = float(p_free_vec[i])
        return p

    def clamp(q: float) -> float:
        cap = float(fan_q_cap)
        if q > cap:
            return cap
        if q < -cap:
            return -cap
        return q

    # d vector (n)
    d_all = np.zeros(n)
    for nid, v in d.items():
        d_all[node_index[nid]] = float(v)

    A_free = A[[node_index[nid] for nid in free_nodes], :]
    d_free = d_all[[node_index[nid] for nid in free_nodes]]

    # dp = -(A^T p_all)
    # d(dp)/d(p_free) = B
    S = np.zeros((n, n_free))
    for nid, i in free_index.items():
        S[node_index[nid], i] = 1.0
    B = -(A.T @ S)  # (m, n_free)

    converged = False
    res_norm = 0.0
    it = 0

    for it in range(1, max_iter + 1):
        p_free = x[:n_free]
        Qfan = x[n_free:]

        p_all = build_p_all(p_free)
        dp = -(A.T @ p_all)

        # flows
        q = np.zeros(m)

        for j, e in enumerate(edges):
            if e.fan_poly is None:
                # effective r includes terminal + damper
                r_eff = float(e.r)

                if e.terminal_orifice is not None:
                    r_eff += _terminal_r(e.terminal_orifice)

                r_eff += _damper_r(e.damper_k, e.damper_u)

                eff = float(dp[j]) + float(e.h)
                effa = max(abs(eff), 1e-12)
                q[j] = float(np.sign(eff) * np.sqrt(effa / r_eff))
            else:
                q[j] = clamp(float(Qfan[fan_pos[j]]))

        # residuals
        F_cont = A_free @ q - d_free

        F_fan = np.zeros(k_fan)
        for ej, pos in fan_pos.items():
            e = edges[ej]
            r = float(e.r)
            a3, a2, a1, a0 = e.fan_poly
            s = float(e.speed_ratio)

            Q = clamp(float(Qfan[pos]))
            Qeff = Q / s
            h = (s * s) * _fan_poly(Qeff, a3, a2, a1, a0)

            F_fan[pos] = float(dp[ej] - (r * Q * abs(Q) - h))

        F = np.concatenate([F_cont, F_fan])
        res_norm = float(np.linalg.norm(F))

        if res_norm < tol:
            converged = True
            break

        # Jacobian
        J = np.zeros((n_free + k_fan, n_free + k_fan))

        # dq/ddp for resistive edges (fan edges are via Qfan)
        dq_ddp = np.zeros(m)
        for j, e in enumerate(edges):
            if e.fan_poly is not None:
                continue

            r_eff = float(e.r)
            if e.terminal_orifice is not None:
                r_eff += _terminal_r(e.terminal_orifice)
            r_eff += _damper_r(e.damper_k, e.damper_u)

            eff = float(dp[j]) + float(e.h)
            effa = max(abs(eff), 1e-12)
            dq_ddp[j] = float(1.0 / (2.0 * np.sqrt(r_eff * effa)))

        # dF_cont/dp_free
        J[:n_free, :n_free] = A_free @ (np.diag(dq_ddp) @ B)

        # dF_cont/dQfan
        for ej, pos in fan_pos.items():
            J[:n_free, n_free + pos] = A_free[:, ej]

        # dF_fan/dp_free
        for ej, pos in fan_pos.items():
            J[n_free + pos, :n_free] = B[ej, :]

        # dF_fan/dQfan
        for ej, pos in fan_pos.items():
            e = edges[ej]
            r = float(e.r)
            a3, a2, a1, a0 = e.fan_poly
            s = float(e.speed_ratio)

            Q = clamp(float(Qfan[pos]))
            d_r = 2.0 * r * max(abs(Q), 1e-12)

            Qeff = Q / s
            dh = s * _fan_poly_deriv(Qeff, a3, a2, a1)

            J[n_free + pos, n_free + pos] = -(d_r - dh)

        # Newton step
        try:
            dx = np.linalg.solve(J, -F)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(J, -F, rcond=None)[0]

        x = x + float(damping) * dx

        # keep Qfan bounded
        for pos in range(k_fan):
            x[n_free + pos] = clamp(float(x[n_free + pos]))

    # build output
    p_free = x[:n_free]
    Qfan = x[n_free:]
    p_all = build_p_all(p_free)
    dp = -(A.T @ p_all)

    q = np.zeros(m)
    for j, e in enumerate(edges):
        if e.fan_poly is None:
            r_eff = float(e.r)
            if e.terminal_orifice is not None:
                r_eff += _terminal_r(e.terminal_orifice)
            r_eff += _damper_r(e.damper_k, e.damper_u)

            eff = float(dp[j]) + float(e.h)
            effa = max(abs(eff), 1e-12)
            q[j] = float(np.sign(eff) * np.sqrt(effa / r_eff))
        else:
            q[j] = float(Qfan[fan_pos[j]])

    return SolveResult(
        p={nid: float(p_all[node_index[nid]]) for nid in node_ids},
        q={edges[j].id: float(q[j]) for j in range(m)},
        converged=converged,
        iters=it,
        residual_norm=res_norm,
    )
