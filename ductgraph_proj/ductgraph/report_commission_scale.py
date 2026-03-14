from __future__ import annotations

from typing import Dict, Iterable, List, Set, Tuple

from ductgraph.commissioning_scale import CommissionScaleResult, ScalingCaseResult
from ductgraph.model import Network
from ductgraph.power_curve import PowerPoly3, compute_electric_power_from_shaft_poly


def fmt_table(headers: Iterable[str], rows: Iterable[Iterable[str]]) -> str:
    # Robust table formatter: pads uneven rows to prevent IndexError.
    hs = [str(h) for h in headers]
    rs = [[str(c) for c in row] for row in rows]

    max_cols = len(hs)
    for row in rs:
        if len(row) > max_cols:
            max_cols = len(row)

    if len(hs) < max_cols:
        hs.extend([""] * (max_cols - len(hs)))

    for row in rs:
        if len(row) < max_cols:
            row.extend([""] * (max_cols - len(row)))

    widths = [0] * max_cols
    for i, c in enumerate(hs):
        widths[i] = max(widths[i], len(c))
    for row in rs:
        for i, c in enumerate(row):
            widths[i] = max(widths[i], len(c))

    def _is_num(x: str) -> bool:
        try:
            float(x)
            return True
        except Exception:
            return False

    def _fmt_row(row: List[str]) -> str:
        out = []
        for i, c in enumerate(row):
            out.append(c.rjust(widths[i]) if _is_num(c) else c.ljust(widths[i]))
        return "  ".join(out)

    lines = [_fmt_row(hs), "  ".join("-" * w for w in widths)]
    lines.extend(_fmt_row(row) for row in rs)
    return "\n".join(lines)


def _fmt_e(x: float) -> str:
    try:
        return f"{float(x):.2e}"
    except Exception:
        return "nan"


def _max_err_cmh(case: ScalingCaseResult) -> float:
    # case.max_err is in m3/s. convert to cmh.
    try:
        return float(getattr(case, "max_err", float("nan"))) * 3600.0
    except Exception:
        return float("nan")


def _build_active_map(scaling_cases: List[Tuple[str, List[int]]]) -> Dict[str, Set[int]]:
    return {str(name): {int(eid) for eid in eids} for name, eids in scaling_cases}


def _reason(
    case: ScalingCaseResult,
    *,
    active_map: Dict[str, Set[int]],
    all_terms: List[int],
    q_design: Dict[int, float],
    warn_ratio_th: float,
) -> str:
    conv = bool(getattr(case.res, "converged", False))
    if not conv:
        return "NC"
    if not bool(getattr(case, "ok", False)):
        return "FAIL"

    active = active_map.get(str(case.name), set(all_terms))
    warn = False
    for eid in active:
        qd = float(q_design.get(eid, float("nan")))
        if not (qd == qd) or qd <= 0:
            continue
        q = abs(float(case.res.q.get(eid, 0.0)))
        ratio = q / qd
        if ratio < warn_ratio_th:
            warn = True
    return "WARN" if warn else "OK"


def build_summary_tables(
    *,
    result: CommissionScaleResult,
    scaling_cases: List[Tuple[str, List[int]]],
    q_design: Dict[int, float],
    net: Network,
    fan_edge_ids: List[int],
    base_hz: float,
    shaft_poly: PowerPoly3,
    eta_total: float,
    eps_under_rel: float,
    tol_warn_rel: float,
) -> Tuple[str, str, Dict[str, Set[int]], List[int]]:
    active_map = _build_active_map(scaling_cases)
    all_terms = sorted(q_design.keys())
    under_ratio_th = 1.0 - float(eps_under_rel)
    warn_ratio_th = max(under_ratio_th, 1.0 - float(tol_warn_rel))

    rows1: List[List[str]] = []
    rows2: List[List[str]] = []

    for case in result.cases:
        s_ratio = float(getattr(case, "speed_ratio", float("nan")))
        hz = float(base_hz) * s_ratio

        q_fan_cmh = float("nan")
        p_elec = float("nan")
        try:
            pw = compute_electric_power_from_shaft_poly(
                net,
                case.res,
                fan_edge_ids=fan_edge_ids,
                speed_ratio=s_ratio,
                shaft_poly=shaft_poly,
                eta_total=float(eta_total),
            )
            q_fan_cmh = sum(abs(float(f.Q_m3s)) for f in pw.fans) * 3600.0
            p_elec = float(pw.elec_total_kw)
        except ValueError:
            # non-converged case etc: keep NaN for display table
            pass

        conv = bool(getattr(case.res, "converged", False))
        iters = int(getattr(case.res, "iters", -1))
        resnorm = float(getattr(case.res, "residual_norm", float("nan")))
        reason = _reason(
            case,
            active_map=active_map,
            all_terms=all_terms,
            q_design=q_design,
            warn_ratio_th=warn_ratio_th,
        )

        rows1.append([
            str(case.name),
            f"{hz:4.1f}",
            f"{q_fan_cmh:7.0f}",
            f"{p_elec:6.2f}",
            "OK" if bool(getattr(case, "ok", False)) else "NG",
            reason,
        ])
        rows2.append([
            str(case.name),
            "T" if conv else "F",
            f"{iters:3d}",
            f"{_max_err_cmh(case):7.0f}",
            _fmt_e(resnorm),
        ])

    table1 = fmt_table(["case", "Hz", "Qfan[cmh]", "Pelec[kW]", "ok", "reason"], rows1)
    table2 = fmt_table(["case", "conv", "it", "maxErr[cmh]", "resnorm"], rows2)
    return table1, table2, active_map, all_terms


def build_terminal_table(
    *,
    case: ScalingCaseResult,
    q_design: Dict[int, float],
    active_edges: Set[int],
    all_terms: List[int],
    eps_under_rel: float,
    tol_warn_rel: float,
    theta_off_deg: float = 1.0,
) -> str:
    conv = bool(getattr(case.res, "converged", False))
    under_ratio_th = 1.0 - float(eps_under_rel)
    warn_ratio_th = max(under_ratio_th, 1.0 - float(tol_warn_rel))

    term_rows: List[List[str]] = []
    for eid in all_terms:
        qd_m3s = float(q_design.get(eid, float("nan")))
        qd_cmh = qd_m3s * 3600.0 if qd_m3s == qd_m3s else float("nan")

        q_m3s = abs(float(case.res.q.get(eid, 0.0)))
        q_cmh = q_m3s * 3600.0
        ratio = (q_m3s / qd_m3s) if (qd_m3s == qd_m3s and abs(qd_m3s) > 0) else float("nan")

        is_off = eid not in active_edges
        if is_off:
            theta = float(theta_off_deg)
            flag = "OFF"
        else:
            theta = float(getattr(case, "thetas", {}).get(eid, float("nan")))
            if not conv:
                flag = "NC"
            elif ratio == ratio and ratio < under_ratio_th:
                flag = "UNDER"
            elif ratio == ratio and ratio < warn_ratio_th:
                flag = "WARN"
            else:
                flag = "OK"

        term_rows.append([
            str(eid),
            f"{theta:.1f}" if theta == theta else "nan",
            f"{qd_cmh:.0f}" if qd_cmh == qd_cmh else "nan",
            f"{q_cmh:.1f}" if q_cmh == q_cmh else "nan",
            f"{ratio:.3f}" if ratio == ratio else "nan",
            flag,
        ])

    return fmt_table(["edge", "theta[deg]", "Qdes[cmh]", "Q[cmh]", "Q/Qdes", "flag"], term_rows)
