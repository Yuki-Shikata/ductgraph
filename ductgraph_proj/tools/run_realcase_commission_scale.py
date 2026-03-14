from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from pprint import pprint

# Ensure project root is importable even when executed from tools/
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


# --- trace: count solve_node_head calls without modifying solver ---
TRACE_SOLVER_CALLS = os.environ.get("DGRAPH_TRACE_SOLVER_CALLS", "1") == "1"
_SOLVE_CALLS = 0


def _install_solve_counter_wrapper():
    global _SOLVE_CALLS
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


from cases.real_case import (
    CAV_EDGES,
    EPS_UNDER_REL_DEFAULT,
    TOL_WARN_REL_DEFAULT,
    FAN_EDGE_IDS,
    FIXED_P,
    Q_DESIGN,
    REALCASE_DAMPER_GAMMA_DEFAULT,
    REALCASE_DAMPER_MODEL_DEFAULT,
    REALCASE_FAN_POLY_DEFAULT,
    REALCASE_HZ_MIN_DEFAULT,
    REALCASE_HZ_MAX_DEFAULT,
    REALCASE_THETA_CENTER_DEFAULT,
    REALCASE_THETA_BAND_DEFAULT,
    REALCASE_THETA_OFF_DEFAULT,
    REALCASE_OFF_DAMPER_U_DEFAULT,
    SCALING_CASES,
    make_net,
)
from ductgraph.commissioning_scale import commission_and_scale
from ductgraph.power_curve import PowerPoly3
from ductgraph.report_commission_scale import build_summary_tables, build_terminal_table


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(REALCASE_DAMPER_MODEL_DEFAULT), help="damper model: sin|linear|exp|expk")
    ap.add_argument("--gamma", type=float, default=float(REALCASE_DAMPER_GAMMA_DEFAULT), help="gamma (sin/linear exponent or exp/expk strength)")
    ap.add_argument("--tol-q", type=float, default=3e-3, help="abs tolerance for |Q|-Qdes match")
    ap.add_argument("--eps-under-rel", type=float, default=float(EPS_UNDER_REL_DEFAULT), help="acceptance margin (equipment tolerance), e.g. 0.01=1%")
    ap.add_argument("--tol-warn-rel", type=float, default=float(TOL_WARN_REL_DEFAULT), help="display threshold (effective WARN threshold=max(1-eps_under_rel, 1-tol_warn_rel))")
    ap.add_argument("--fan-q-init", type=float, default=2.0)
    ap.add_argument("--fan-q-cap", type=float, default=80.0)

    # --- fan performance curve (base speed) ---
    ap.add_argument("--fan-a", type=float, default=float(REALCASE_FAN_POLY_DEFAULT[0]), help="fan poly a (Q^3), Q in m3/s, dp in Pa")
    ap.add_argument("--fan-b", type=float, default=float(REALCASE_FAN_POLY_DEFAULT[1]), help="fan poly b (Q^2), Q in m3/s, dp in Pa")
    ap.add_argument("--fan-c", type=float, default=float(REALCASE_FAN_POLY_DEFAULT[2]), help="fan poly c (Q), Q in m3/s, dp in Pa")
    ap.add_argument("--fan-d", type=float, default=float(REALCASE_FAN_POLY_DEFAULT[3]), help="fan poly d (const), dp in Pa")

    # --- power curve (base speed) ---
    ap.add_argument("--pw-a", type=float, default=-0.000004, help="shaft power poly a (Q^3), Q in cmm")
    ap.add_argument("--pw-b", type=float, default=0.0007, help="shaft power poly b (Q^2), Q in cmm")
    ap.add_argument("--pw-c", type=float, default=-0.008, help="shaft power poly c (Q), Q in cmm")
    ap.add_argument("--pw-d", type=float, default=0.404, help="shaft power poly d (const), kW")
    ap.add_argument("--eta-total", type=float, default=0.60, help="total efficiency (0<eta<=1). P_elec=P_shaft/eta")
    ap.add_argument("--base-hz", type=float, default=50.0, help="base inverter frequency [Hz] for reporting. freq = base_hz * speed_ratio")
    ap.add_argument("--hz-min", type=float, default=None, help="minimum inverter frequency [Hz] for commissioning search")
    ap.add_argument("--hz-max", type=float, default=None, help="maximum inverter frequency [Hz] for commissioning search")
    ap.add_argument("--theta-center", type=float, default=float(REALCASE_THETA_CENTER_DEFAULT), help="full-load target opening center [deg]")
    ap.add_argument("--theta-band", type=float, default=float(REALCASE_THETA_BAND_DEFAULT), help="full-load target opening half-band [deg]")
    ap.add_argument("--off-theta", type=float, default=float(REALCASE_THETA_OFF_DEFAULT), help="OFF terminal angle [deg] when off-u is disabled")
    ap.add_argument("--off-u", type=float, default=float(REALCASE_OFF_DAMPER_U_DEFAULT), help="OFF terminal fixed opening ratio u in (0,1]; set <=0 to disable")
    # Backward-compatible options (hidden): use Hz options for new workflows.
    ap.add_argument("--s-min", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--s-max", type=float, default=None, help=argparse.SUPPRESS)

    args = ap.parse_args()

    base_hz = float(args.base_hz)
    if base_hz <= 0.0:
        raise ValueError("base_hz must be > 0")

    default_s_min = float(REALCASE_HZ_MIN_DEFAULT) / base_hz
    default_s_max = float(REALCASE_HZ_MAX_DEFAULT) / base_hz

    if args.hz_min is not None:
        s_min = float(args.hz_min) / base_hz
    elif args.s_min is not None:
        s_min = float(args.s_min)
    else:
        s_min = default_s_min

    if args.hz_max is not None:
        s_max = float(args.hz_max) / base_hz
    elif args.s_max is not None:
        s_max = float(args.s_max)
    else:
        s_max = default_s_max

    if s_min <= 0.0:
        raise ValueError("effective s_min must be > 0")
    if s_max <= s_min:
        raise ValueError("effective s_max must be > s_min")

    hz_min = s_min * base_hz
    hz_max = s_max * base_hz

    if float(args.theta_band) <= 0.0:
        raise ValueError("theta_band must be > 0")
    if not (0.0 <= float(args.off_theta) <= 90.0):
        raise ValueError("off_theta must be in [0, 90]")

    effective_off_u = None
    if float(args.off_u) > 0.0:
        effective_off_u = float(args.off_u)
        if not (0.0 < effective_off_u <= 1.0):
            raise ValueError("off_u must be in (0, 1] when enabled")

    if TRACE_SOLVER_CALLS:
        _install_solve_counter_wrapper()
        global _SOLVE_CALLS
        _SOLVE_CALLS = 0

    fan_poly = (float(args.fan_a), float(args.fan_b), float(args.fan_c), float(args.fan_d))
    net = make_net(
        damper_model=str(args.model),
        damper_gamma=float(args.gamma),
        fan_poly=fan_poly,
    )

    params = dict(
        fan_edge_ids=list(FAN_EDGE_IDS),
        full_active_cav_edge_ids=list(CAV_EDGES),
        q_design_by_edge=dict(Q_DESIGN),
        scaling_cases=list(SCALING_CASES),
        fixed_p=dict(FIXED_P),
        d={},
        model=str(args.model),
        gamma=float(args.gamma),
        tol_q=float(args.tol_q),
        fan_q_init=float(args.fan_q_init),
        fan_q_cap=float(args.fan_q_cap),
        s_min=float(s_min),
        s_max=float(s_max),
        theta_center=float(args.theta_center),
        theta_band=float(args.theta_band),
        theta_off_deg=float(args.off_theta),
        off_damper_u=effective_off_u,
        eps_under_rel=float(args.eps_under_rel),
    )

    if os.environ.get("DGRAPH_PRINT_INPUTS", "1") == "1":
        print("=== inputs (commission_and_scale) ===")
        params_view = dict(params)
        # hide compatibility seed and internal speed-ratio bounds from operator-facing input display
        params_view.pop("s_min", None)
        params_view.pop("s_max", None)
        pprint(
            dict(
                params_view,
                hz_min=float(hz_min),
                hz_max=float(hz_max),
                fan_curve_base=dict(
                    a=float(args.fan_a),
                    b=float(args.fan_b),
                    c=float(args.fan_c),
                    d=float(args.fan_d),
                ),
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
        " converged_full:", out.res_full.converged,
        " index_edge_id:", out.index_edge_id,
        " full_load_band_ok:", out.full_load_band_ok,
        " full_load_index_theta:", out.full_load_index_theta,
        " full_load_diag:", out.full_load_diag,
        " full_load_critical_edges:", out.full_load_critical_edges,
        " full_load_critical_thetas:", out.full_load_critical_thetas,
        " full_load_diag_msg:", out.full_load_diag_msg,
    )

    if not out.cases:
        print("NO cases returned.")
        return 0

    print("\n=== Summary (cases) ===")
    shaft_poly = PowerPoly3(
        a=float(args.pw_a),
        b=float(args.pw_b),
        c=float(args.pw_c),
        d=float(args.pw_d),
        f0_hz=float(base_hz),
    )

    table1, table2, active_map, all_terms = build_summary_tables(
        result=out,
        scaling_cases=list(SCALING_CASES),
        q_design=dict(Q_DESIGN),
        net=net,
        fan_edge_ids=list(FAN_EDGE_IDS),
        base_hz=float(base_hz),
        shaft_poly=shaft_poly,
        eta_total=float(args.eta_total),
        eps_under_rel=float(args.eps_under_rel),
        tol_warn_rel=float(args.tol_warn_rel),
    )
    print(table1)
    print(table2)

    for case in out.cases:
        print(f"\n=== Terminals ({case.name}) ===")
        active_edges = active_map.get(str(case.name), set(all_terms))
        print(
            build_terminal_table(
                case=case,
                q_design=dict(Q_DESIGN),
                active_edges=active_edges,
                all_terms=all_terms,
                eps_under_rel=float(args.eps_under_rel),
                tol_warn_rel=float(args.tol_warn_rel),
                theta_off_deg=float(args.off_theta),
            )
        )

    if TRACE_SOLVER_CALLS:
        print(f"\n[trace] solve_node_head calls: {_SOLVE_CALLS}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
