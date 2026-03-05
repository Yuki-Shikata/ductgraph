from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from pprint import pprint
from typing import Any

# Ensure project root is importable even when executed from tools/
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


# --- trace: count solve_node_head calls without modifying solver ---
TRACE_SOLVER_CALLS = os.environ.get("DGRAPH_TRACE_SOLVER_CALLS", "1") == "1"
PRETTY = os.environ.get("DGRAPH_PRETTY", "1") == "1"
_SOLVE_CALLS = 0

def _install_solve_counter_wrapper():
    global _SOLVE_CALLS
    import sys
    import ductgraph.solver_nodehead as sn

    orig = sn.solve_node_head

    def wrapped(*args, **kwargs):
        global _SOLVE_CALLS
        _SOLVE_CALLS += 1
        return orig(*args, **kwargs)

    # 1) module attribute
    sn.solve_node_head = wrapped

    # 2) patch already-imported ductgraph.* modules that grabbed solve_node_head via "from ... import"
    for name, mod in list(sys.modules.items()):
        if not name.startswith("ductgraph."):
            continue
        if mod is None:
            continue
        try:
            if getattr(mod, "solve_node_head", None) is orig:
                setattr(mod, "solve_node_head", wrapped)
        except Exception:
            pass

    return orig

from ductgraph.commissioning_scale import commission_and_scale
from cases.real_case import (
    make_net,
    FIXED_P,
    Q_DESIGN,
    CAV_EDGES,
    FAN_EDGE_IDS,
    EDGE_A,
    EDGE_C,
    EDGE_D,
    choose_maxload_edge,
)


def _m3s_to_cmm(q_m3s: float) -> float:
    # m^3/s -> m^3/min (cmm)
    return float(q_m3s) * 60.0


def _shaft_power_kw_poly_cmm(q_cmm: float, *, a: float, b: float, c: float, d: float, speed_ratio: float) -> float:
    """
    Base (speed_ratio=1) shaft power curve:
        P0(Q) = a Q^3 + b Q^2 + c Q + d   [kW], Q in cmm.

    Affinity scaling (Q ~ s, P ~ s^3):
        P(Q,s) = s^3 * P0(Q/s)
               = a*Q^3 + (b*s)*Q^2 + (c*s^2)*Q + (d*s^3)
    """
    Q = float(q_cmm)
    s = float(speed_ratio)
    return (a * Q**3) + ((b * s) * Q**2) + ((c * s**2) * Q) + (d * s**3)


def _fan_power_from_res(
    *,
    res: Any,
    fan_edge_ids: list[int],
    speed_ratio: float,
    a: float,
    b: float,
    c: float,
    d: float,
    eta_total: float,
) -> dict[str, float]:
    """
    Returns dict with:
      q_fan_cmm, p_shaft_kw, p_elec_kw
    """
    eta = float(eta_total)
    if not (0.0 < eta <= 1.0):
        raise ValueError("eta_total must be in (0, 1].")

    q_sum_m3s = 0.0
    for fe in fan_edge_ids:
        q_sum_m3s += abs(float(res.q[fe]))

    q_cmm = _m3s_to_cmm(q_sum_m3s)
    p_shaft = _shaft_power_kw_poly_cmm(q_cmm, a=a, b=b, c=c, d=d, speed_ratio=speed_ratio)
    p_elec = p_shaft / eta
    return {"q_fan_cmm": q_cmm, "p_shaft_kw": p_shaft, "p_elec_kw": p_elec}



def _fmt_table(hs, rs):
    # Robust table formatter: pads uneven rows to prevent IndexError
    hs = list(hs)
    rs = [list(r) for r in rs]

    max_cols = len(hs)
    for r in rs:
        if len(r) > max_cols:
            max_cols = len(r)

    if len(hs) < max_cols:
        hs = hs + [""] * (max_cols - len(hs))
    rs2 = []
    for r in rs:
        if len(r) < max_cols:
            r = r + [""] * (max_cols - len(r))
        rs2.append(r)
    rs = rs2

    widths = [0] * max_cols
    for i, c in enumerate(hs):
        widths[i] = max(widths[i], len(str(c)))
    for r in rs:
        for i, c in enumerate(r):
            widths[i] = max(widths[i], len(str(c)))

    def is_num(x):
        try:
            float(str(x))
            return True
        except Exception:
            return False

    def fmt_row(r):
        out = []
        for i, c in enumerate(r):
            c = str(c)
            out.append(c.rjust(widths[i]) if is_num(c) else c.ljust(widths[i]))
        return "  ".join(out)

    lines = [fmt_row(hs), "  ".join("-" * w for w in widths)]
    lines += [fmt_row(r) for r in rs]
    return "\n".join(lines)

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="sin", help="damper model: sin|linear|exp|expk")
    ap.add_argument("--gamma", type=float, default=1.0, help="gamma (sin/linear exponent or exp/expk strength)")
    ap.add_argument("--tol-q", type=float, default=3e-3, help="abs tolerance for |Q|-Qdes match")
    ap.add_argument("--eps-under-rel", type=float, default=1e-4, help="tiny numeric absorb for acceptance (not a display tolerance)")
    ap.add_argument("--tol-warn-rel", type=float, default=5e-3, help="display tolerance: WARN if Q/Qdes < 1, UNDER if Q/Qdes < 1-tol")
    ap.add_argument("--fan-q-init", type=float, default=2.0)
    ap.add_argument("--fan-q-cap", type=float, default=80.0)

    # --- power curve (base speed) ---
    # ここはあなたの画像の a,b,c,d に合わせて変えてOK（CLIでも上書き可能）
    ap.add_argument("--pw-a", type=float, default=-0.000004, help="shaft power poly a (Q^3), Q in cmm")
    ap.add_argument("--pw-b", type=float, default=0.0007, help="shaft power poly b (Q^2), Q in cmm")
    ap.add_argument("--pw-c", type=float, default=-0.008, help="shaft power poly c (Q), Q in cmm")
    ap.add_argument("--pw-d", type=float, default=0.404, help="shaft power poly d (const), kW")
    ap.add_argument("--eta-total", type=float, default=0.60, help="total efficiency (0<eta<=1). P_elec=P_shaft/eta")
    ap.add_argument("--base-hz", type=float, default=50.0, help="base inverter frequency [Hz] for reporting. freq = base_hz * speed_ratio")
    ap.add_argument("--s-min", type=float, default=0.2, help="speed_ratio lower bound (e.g., 25/50=0.5)")
    ap.add_argument("--s-max", type=float, default=3.0, help="speed_ratio upper bound (e.g., 80/50=1.6)")

    args = ap.parse_args()


    # trace wrapper install (optional)
    if TRACE_SOLVER_CALLS:
        _install_solve_counter_wrapper()
        global _SOLVE_CALLS
        _SOLVE_CALLS = 0

    net = make_net()
    r_by_edge = {e.id: float(e.r) for e in net.edges}
    maxload_edge_id = int(choose_maxload_edge(Q_DESIGN, r_by_edge))

    full_active = list(CAV_EDGES)

    # scaling cases (prefer cases.real_case.SCALING_CASES)
    try:
        from cases.real_case import SCALING_CASES as REAL_SCALING_CASES
        scaling_cases = list(REAL_SCALING_CASES)
    except Exception:
        scaling_cases = None

    if not scaling_cases:
        scaling_cases = [
            ("all", [EDGE_A, EDGE_C, EDGE_D]),
            ("a", [EDGE_A]),
            ("c", [EDGE_C]),
            ("d", [EDGE_D]),
            ("ac", [EDGE_A, EDGE_C]),
            ("ad", [EDGE_A, EDGE_D]),
            ("cd", [EDGE_C, EDGE_D]),
        ]
    # ---- inputs + call params (single source of truth) ----
    params = dict(
        fan_edge_ids=list(FAN_EDGE_IDS),
        maxload_edge_id=maxload_edge_id,
        full_active_cav_edge_ids=full_active,
        q_design_by_edge=dict(Q_DESIGN),
        scaling_cases=scaling_cases,
        fixed_p=dict(FIXED_P),
        d={},
        model=str(args.model),
        gamma=float(args.gamma),
        tol_q=float(args.tol_q),
        fan_q_init=float(args.fan_q_init),
        fan_q_cap=float(args.fan_q_cap),
        s_min=float(args.s_min),
        s_max=float(args.s_max),
        eps_under_rel=float(args.eps_under_rel),
    )

    if os.environ.get("DGRAPH_PRINT_INPUTS", "1") == "1":
        print("=== inputs (commission_and_scale) ===")
        pprint(
            dict(
                params,
                power_curve_base=dict(
                    a=float(args.pw_a),
                    b=float(args.pw_b),
                    c=float(args.pw_c),
                    d=float(args.pw_d),
                ),
                eta_total=float(args.eta_total),
            )
        )

    out = commission_and_scale(net, **params)

    print("=== CommissionScaleResult ===")
    print("model:", args.model, "gamma:", args.gamma)
    print(
        "speed_full:", out.speed_full,
        " q65:", out.q65,
        " q85:", out.q85,
        " converged_full:", out.res_full.converged,
    )

    if not out.cases:
        print("NO cases returned.")
        return 0
    # ---- pretty summary tables (ALL cases) ----
    # (commission_and_scale returns cases with c.res populated)
    case_active = {str(n): list(es) for (n, es) in scaling_cases}
    print("\n=== Summary (cases) ===")

    # ----- helpers (display only) -----
    def _fmt_e(x: float) -> str:
        try:
            return f"{float(x):.2e}"
        except Exception:
            return "nan"

    def _max_err_cmh(case) -> float:
        # case.max_err is in m3/s. convert to cmh.
        try:
            return float(getattr(case, "max_err", float("nan"))) * 3600.0
        except Exception:
            return float("nan")

    # active map from scaling_cases
    case_active = {str(n): list(es) for (n, es) in scaling_cases}
    active_map = {k: set(v) for k, v in case_active.items()}
    all_terms = sorted(Q_DESIGN.keys())

    def _reason(cc) -> str:
        conv = bool(getattr(cc.res, "converged", False))
        if not conv:
            return "NC"
        a = active_map.get(str(cc.name), set(all_terms))
        under = False
        warn = False
        for eid in a:
            qd = float(Q_DESIGN.get(eid, float("nan")))
            if not (qd == qd) or qd <= 0:
                continue
            q = abs(float(cc.res.q.get(eid, 0.0)))
            r = q / qd
            if r < (1.0 - float(args.tol_warn_rel)):
                under = True
            elif r < 1.0:
                warn = True
        return "UNDER" if under else ("WARN" if warn else "OK")

    # ----- summary (2 lines) -----
    rows1, rows2 = [], []
    for cc in out.cases:
        s_ratio = float(getattr(cc, "speed_ratio", float("nan")))
        hz = float(args.base_hz) * s_ratio

        q_fan_m3s = 0.0
        for fe in FAN_EDGE_IDS:
            q_fan_m3s += abs(float(cc.res.q.get(fe, 0.0)))
        q_fan_cmh = q_fan_m3s * 3600.0

        q_fan_cmm = q_fan_m3s * 60.0
        a0 = float(args.pw_a); b0 = float(args.pw_b); c0 = float(args.pw_c); d0 = float(args.pw_d)
        Q = float(q_fan_cmm)
        p_shaft = (a0 * Q**3) + ((b0 * s_ratio) * Q**2) + ((c0 * s_ratio**2) * Q) + (d0 * s_ratio**3)
        p_elec = p_shaft / float(args.eta_total)

        conv = bool(getattr(cc.res, "converged", False))
        iters = int(getattr(cc.res, "iters", -1))
        resnorm = float(getattr(cc.res, "residual_norm", float("nan")))
        reason = _reason(cc)

        rows1.append([
            str(cc.name),
            f"{hz:4.1f}",
            f"{q_fan_cmh:7.0f}",
            f"{p_elec:6.2f}",
            "OK" if bool(getattr(cc, "ok", False)) else "NG",
            reason,
        ])
        rows2.append([
            str(cc.name),
            "T" if conv else "F",
            f"{iters:3d}",
            f"{_max_err_cmh(cc):7.0f}",
            _fmt_e(resnorm),
        ])

    print(_fmt_table(["case","Hz","Qfan[cmh]","Pelec[kW]","ok","reason"], rows1))
    print(_fmt_table(["case","conv","it","maxErr[cmh]","resnorm"], rows2))

    # ----- terminals (always show a/c/d; OFF marked) -----
    for cc in out.cases:
        print(f"\n=== Terminals ({cc.name}) ===")
        conv = bool(getattr(cc.res, "converged", False))
        a = active_map.get(str(cc.name), set(all_terms))

        term_rows = []
        for eid in all_terms:
            qd_m3s = float(Q_DESIGN.get(eid, float("nan")))
            qd_cmh = qd_m3s * 3600.0 if qd_m3s == qd_m3s else float("nan")

            q_m3s = abs(float(cc.res.q.get(eid, 0.0)))
            q_cmh = q_m3s * 3600.0
            ratio = (q_m3s / qd_m3s) if (qd_m3s == qd_m3s and abs(qd_m3s) > 0) else float("nan")

            is_off = eid not in a
            if is_off:
                th = 1.0  # theta_off 表示（見やすさ優先）
                flag = "OFF"
            else:
                th = float(getattr(cc, "thetas", {}).get(eid, float("nan")))
                if not conv:
                    flag = "NC"
                else:
                    if ratio == ratio and ratio < (1.0 - float(args.tol_warn_rel)):
                        flag = "UNDER"
                    elif ratio == ratio and ratio < 1.0:
                        flag = "WARN"
                    else:
                        flag = "OK"

            term_rows.append([
                str(eid),
                f"{th:.1f}" if th == th else "nan",
                f"{qd_cmh:.0f}" if qd_cmh == qd_cmh else "nan",
                f"{q_cmh:.1f}" if q_cmh == q_cmh else "nan",
                f"{ratio:.3f}" if ratio == ratio else "nan",
                flag,
            ])

        print(_fmt_table(["edge","theta[deg]","Qdes[cmh]","Q[cmh]","Q/Qdes","flag"], term_rows))


if __name__ == "__main__":
    raise SystemExit(main())
