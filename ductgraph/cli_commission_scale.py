from __future__ import annotations

import argparse
import importlib
from typing import Dict, List, Tuple

from ductgraph.commissioning_scale import commission_and_scale


def _parse_int_list(s: str) -> List[int]:
    return [int(x.strip()) for x in s.split(",") if x.strip()]


def _parse_map(s: str) -> Dict[int, float]:
    """
    "1:1.0,2:0.8" -> {1:1.0,2:0.8}
    """
    out: Dict[int, float] = {}
    if not s.strip():
        return out
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        k, v = part.split(":")
        out[int(k.strip())] = float(v.strip())
    return out


def _parse_cases(s: str) -> List[Tuple[str, List[int]]]:
    """
    "caseA=1,2,3;caseB=1,3" -> [("caseA",[1,2,3]),("caseB",[1,3])]
    """
    out: List[Tuple[str, List[int]]] = []
    s = s.strip()
    if not s:
        return out
    for block in s.split(";"):
        block = block.strip()
        if not block:
            continue
        name, rhs = block.split("=")
        eids = _parse_int_list(rhs)
        out.append((name.strip(), eids))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Commission(full-load with maxload band) + Scaling(CAV 0..90 by Broyden).")

    ap.add_argument("--factory", required=True, help="Network factory path: module:function (returns Network)")
    ap.add_argument("--fan_edges", required=True, help='fan edge ids. e.g. "0" or "0,5"')
    ap.add_argument("--maxload_edge", required=True, type=int, help="maxload terminal edge id")
    ap.add_argument("--full_active", required=True, help='full-load active cav edges. e.g. "1,2,3"')
    ap.add_argument("--q_design", required=True, help='design q per edge. e.g. "1:1.0,2:1.0,3:1.0"')

    ap.add_argument("--cases", required=True, help='scaling cases: "A=1,2,3;B=1,3;C=2"')

    ap.add_argument("--fixed_p", default="", help='fixed node pressures e.g. "0:0,2:0,3:0" (omit -> ref_node=0)')
    ap.add_argument("--d", default="", help='external flow d e.g. "0:-2.0,5:1.0" (omit -> {})')

    ap.add_argument("--theta_center", type=float, default=75.0)
    ap.add_argument("--theta_band", type=float, default=10.0)

    ap.add_argument("--model", default="sin", choices=["sin", "linear", "exp", "expk"])
    ap.add_argument("--gamma", type=float, default=1.0)

    ap.add_argument("--s_min", type=float, default=0.2)
    ap.add_argument("--s_max", type=float, default=3.0)
    ap.add_argument("--tol_q", type=float, default=3e-3)
    ap.add_argument("--eps_under_rel", type=float, default=2e-4)

    ap.add_argument("--fan_q_init", type=float, default=2.0)
    ap.add_argument("--fan_q_cap", type=float, default=80.0)

    args = ap.parse_args()

    mod_name, func_name = args.factory.split(":")
    mod = importlib.import_module(mod_name)
    make_net = getattr(mod, func_name)
    net = make_net()

    fixed_p = _parse_map(args.fixed_p)
    if not fixed_p:
        fixed_p = None

    d = _parse_map(args.d)
    if not d:
        d = None

    fan_edges = _parse_int_list(args.fan_edges)
    full_active = _parse_int_list(args.full_active)
    q_design = _parse_map(args.q_design)
    cases = _parse_cases(args.cases)

    out = commission_and_scale(
        net,
        fan_edge_ids=fan_edges,
        maxload_edge_id=args.maxload_edge,
        full_active_cav_edge_ids=full_active,
        q_design_by_edge=q_design,
        scaling_cases=cases,
        theta_center=args.theta_center,
        theta_band=args.theta_band,
        fixed_p=fixed_p,
        d=d,
        model=args.model,
        gamma=args.gamma,
        s_min=args.s_min,
        s_max=args.s_max,
        tol_q=args.tol_q,
        eps_under_rel=args.eps_under_rel,
        fan_q_init=args.fan_q_init,
        fan_q_cap=args.fan_q_cap,
    )

    print("=== FULL LOAD ===")
    print(f"speed_full={out.speed_full:.6g}  maxload band q65={out.q65:.6g} q85={out.q85:.6g}  converged={out.res_full.converged}")

    print("\n=== SCALING CASES ===")
    for c in out.cases:
        print(f"\n-- {c.name} --")
        print(f"speed={c.speed_ratio:.6g} ok={c.ok} msg={c.msg} max_err={c.max_err:.6g} converged={c.res.converged}")
        if c.saturated_90:
            print("theta>=90deg edges:", ",".join(str(x) for x in sorted(c.saturated_90)))
        for eid, th in sorted(c.thetas.items()):
            qabs = c.achieved_q_abs.get(eid, float("nan"))
            print(f"edge {eid}: theta={th:.6g}deg  absq={qabs:.6g}  target={q_design[eid]:.6g}  err={qabs-q_design[eid]:+.6g}")


if __name__ == "__main__":
    main()
