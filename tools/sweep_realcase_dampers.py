from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Dict, List, Tuple
from pathlib import Path
import sys

# Ensure project root importable
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ductgraph.control_damper import with_damper_angle
from ductgraph.solver_nodehead import solve_node_head
from cases.real_case import (
    make_net, FIXED_P, Q_DESIGN, CAV_EDGES, EDGE_LABELS,
)

@dataclass
class Row:
    name: str
    thetas: Dict[int, float]
    converged: bool
    qabs: Dict[int, float]

def fmt_eid(eid: int) -> str:
    lab = EDGE_LABELS.get(eid, str(eid))
    return f"{eid}({lab})"

def run_case(net, fixed_p, model: str, gamma: float, thetas: Dict[int, float], fan_q_init: float, fan_q_cap: float) -> Tuple[bool, Dict[int, float]]:
    net2 = with_damper_angle(net, thetas, model=model, gamma=gamma)
    res = solve_node_head(net2, d={}, fixed_p=dict(fixed_p), fan_q_init=fan_q_init, fan_q_cap=fan_q_cap)
    qabs = {eid: float(abs(res.q[eid])) for eid in CAV_EDGES} if res.converged else {eid: float("nan") for eid in CAV_EDGES}
    return bool(res.converged), qabs

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="expk", help="sin|linear|exp|expk")
    ap.add_argument("--gamma", type=float, default=5.0)
    ap.add_argument("--fan-q-init", type=float, default=2.0)
    ap.add_argument("--fan-q-cap", type=float, default=80.0)
    ap.add_argument("--eps-under-rel", type=float, default=1e-4)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    net = make_net()
    fixed_p = dict(FIXED_P)
    qdes = dict(Q_DESIGN)
    eps_under_rel = float(args.eps_under_rel)

    # --- 1) 単調性チェック用：各端末を単独で 0->90 を掃引（他は90固定） ---
    sweep_angles = [0.0, 10.0, 20.0, 30.0, 45.0, 60.0, 75.0, 85.0, 90.0]

    rows: List[Row] = []

    for eid in CAV_EDGES:
        base = {e: 90.0 for e in CAV_EDGES}  # others wide open
        for th in sweep_angles:
            thetas = dict(base)
            thetas[eid] = float(th)
            conv, qabs = run_case(net, fixed_p, args.model, args.gamma, thetas, args.fan_q_init, args.fan_q_cap)
            rows.append(Row(name=f"sweep_{eid}_{th:g}", thetas=thetas, converged=conv, qabs=qabs))

    # --- 2) Authority/Capacity 判定用：端末ごとに (0°) と (90°) だけ確認（他は90固定） ---
    diag = {}
    for eid in CAV_EDGES:
        base = {e: 90.0 for e in CAV_EDGES}
        # full open at this terminal (90)
        th_open = dict(base); th_open[eid] = 90.0
        conv_o, q_o = run_case(net, fixed_p, args.model, args.gamma, th_open, args.fan_q_init, args.fan_q_cap)

        # full close at this terminal (0)
        th_cls = dict(base); th_cls[eid] = 0.0
        conv_c, q_c = run_case(net, fixed_p, args.model, args.gamma, th_cls, args.fan_q_init, args.fan_q_cap)

        diag[eid] = (conv_o, q_o[eid], conv_c, q_c[eid])

    # ===== 出力 =====
    print("=== sweep_realcase_dampers ===")
    print(f"model={args.model} gamma={args.gamma} fan_q_init={args.fan_q_init} fan_q_cap={args.fan_q_cap}")
    print("Design Q (m3/s):")
    for eid in CAV_EDGES:
        print(f"  {fmt_eid(eid)}: Qdes={qdes[eid]:.6g}")

    print("\n--- [A] Monotonicity quick check (others 90deg) ---")
    for eid in CAV_EDGES:
        # collect that eid's sweep rows
        sub = [r for r in rows if r.name.startswith(f"sweep_{eid}_")]
        # build theta list and q list
        ths = [r.thetas[eid] for r in sub]
        qs = [r.qabs[eid] for r in sub]
        # check monotonic nondecreasing with theta (should increase as you open)
        mono = True
        for i in range(1, len(qs)):
            if not (qs[i] >= qs[i-1] - 1e-12):
                mono = False
                break
        print(f"\n  {fmt_eid(eid)} monotonic_increase_with_theta: {mono}")
        if args.verbose or (not mono):
            for th, q in zip(ths, qs):
                print(f"    theta={th:>5.1f}  Qabs={q:.6g}")

    print("\n--- [B] Authority / Capacity diagnosis (others 90deg) ---")
    print("Interpretation:")
    print("  - authority 부족: theta=0°でも Q > Qdes+tol (絞り切っても過大)")
    print("  - capacity 不足 : theta=90°でも Q < Qdes*(1-eps_under_rel) (全開でも不足)")
    tol = 3e-3  # same as your commissioning default; used only for this diagnosis
    for eid in CAV_EDGES:
        conv_o, q_open, conv_c, q_close = diag[eid]
        qd = float(qdes[eid])
        q_min_req = qd * (1.0 - eps_under_rel)

        flags: List[str] = []
        if not conv_o or not conv_c:
            flags.append("NOTE: inner solve not converged in some condition")

        # capacity
        if conv_o and (q_open < q_min_req):
            flags.append(f"CAPACITY_WARN: open(90) Q={q_open:.6g} < Qmin={q_min_req:.6g}")

        # authority
        if conv_c and (q_close > (qd + tol)):
            flags.append(f"AUTHORITY_WARN: close(0) Q={q_close:.6g} > Qdes+tol={qd+tol:.6g}")

        print(f"\n  {fmt_eid(eid)}")
        print(f"    open(90):  converged={conv_o}  Qabs={q_open:.6g}")
        print(f"    close(0): converged={conv_c}  Qabs={q_close:.6g}")
        if flags:
            for f in flags:
                print(f"    -> {f}")
        else:
            print("    -> OK (no capacity/authority warning by these criteria)")

    print("\n--- [C] Sanity: one-line suggestion ---")
    print("If monotonicity is False: model/implementation mismatch likely (theta convention or u mapping).")
    print("If AUTHORITY_WARN occurs: increase damper_k and/or gamma, or reduce u_min(leak), or add series loss.")
    print("If CAPACITY_WARN occurs: fan/speed or upstream resistance dominates; even full-open cannot meet design.")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
