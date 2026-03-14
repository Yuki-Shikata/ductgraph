from __future__ import annotations

import argparse
import importlib
import inspect
from typing import Dict, List

from ductgraph.control_cav import solve_cav_dampers_broyden


def _parse_int_list(s: str) -> List[int]:
    return [int(x.strip()) for x in s.split(",") if x.strip()]


def _parse_targets(s: str) -> Dict[int, float]:
    """
    "1:1.0,2:0.8,10:2.5" -> {1:1.0,2:0.8,10:2.5}
    """
    out: Dict[int, float] = {}
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        k, v = part.split(":")
        out[int(k.strip())] = float(v.strip())
    return out



def _build_net_from_factory(factory, *, model: str, gamma: float):
    try:
        sig = inspect.signature(factory)
    except (TypeError, ValueError):
        return factory()

    kwargs = {}
    if "damper_model" in sig.parameters:
        kwargs["damper_model"] = str(model)
    if "damper_gamma" in sig.parameters:
        kwargs["damper_gamma"] = float(gamma)
    return factory(**kwargs)


def main() -> None:
    ap = argparse.ArgumentParser(description="Run CAV damper solver (Broyden) on a provided Network factory.")
    ap.add_argument("--factory", required=True,
                    help="Python path to a function that returns Network. e.g. mycases.case1:make_net")
    ap.add_argument("--fixed_p", default="",
                    help='fixed node pressure. e.g. "0:0,2:0,3:0" (omit to use ref_node=0Pa)')
    ap.add_argument("--d", default="",
                    help='external flow d. e.g. "0:-2.0,5:1.0" (omit for {})')

    ap.add_argument("--cav_edges", required=True, help='comma list of cav edge ids. e.g. "10,11,12"')
    ap.add_argument("--targets", required=True, help='targets per edge. e.g. "10:1.0,11:1.0,12:0.8"')

    ap.add_argument("--theta0", type=float, default=45.0)
    ap.add_argument("--theta_min", type=float, default=0.0)
    ap.add_argument("--theta_max", type=float, default=90.0)

    ap.add_argument("--model", default="sin", choices=["sin", "linear", "exp", "expk"])
    ap.add_argument("--gamma", type=float, default=1.0)

    ap.add_argument("--tol_q", type=float, default=3e-3)
    ap.add_argument("--eps_under_rel", type=float, default=5e-3)
    ap.add_argument("--max_iters", type=int, default=60)
    ap.add_argument("--alpha", type=float, default=0.7)
    ap.add_argument("--H0_gain", type=float, default=1.0)

    ap.add_argument("--fan_q_init", type=float, default=2.0)
    ap.add_argument("--fan_q_cap", type=float, default=80.0)

    args = ap.parse_args()

    # import factory
    mod_name, func_name = args.factory.split(":")
    mod = importlib.import_module(mod_name)
    make_net = getattr(mod, func_name)

    net = _build_net_from_factory(make_net, model=str(args.model), gamma=float(args.gamma))

    fixed_p = None
    if args.fixed_p.strip():
        fixed_p = _parse_targets(args.fixed_p)  # reuse parser (int:float)

    d = None
    if args.d.strip():
        d = _parse_targets(args.d)

    cav_edges = _parse_int_list(args.cav_edges)
    targets = _parse_targets(args.targets)

    out = solve_cav_dampers_broyden(
        net,
        cav_edge_ids=cav_edges,
        targets_q=targets,
        theta0=args.theta0,
        theta_min=args.theta_min,
        theta_max=args.theta_max,
        fixed_p=fixed_p,
        d=d,
        model=args.model,
        gamma=args.gamma,
        tol_q=args.tol_q,
        eps_under_rel=args.eps_under_rel,
        max_iters=args.max_iters,
        alpha=args.alpha,
        H0_gain=args.H0_gain,
        fan_q_init=args.fan_q_init,
        fan_q_cap=args.fan_q_cap,
    )

    print(f"ok={out.ok} msg={out.msg} iters={out.iters} max_err={out.max_err}")
    # thetas
    for eid, th in sorted(out.thetas.items()):
        qabs = abs(out.res.q[eid])
        print(f"edge {eid}: theta={th:.6g}deg  absq={qabs:.6g}  target={targets[eid]:.6g}  err={qabs-targets[eid]:+.6g}")


if __name__ == "__main__":
    main()
