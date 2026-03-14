from cases.real_case import (
    CAV_EDGES,
    FAN_EDGE_IDS,
    FIXED_P,
    Q_DESIGN,
    REALCASE_DAMPER_GAMMA_DEFAULT,
    REALCASE_DAMPER_MODEL_DEFAULT,
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


def _get_case(out, name: str):
    for c in out.cases:
        if str(c.name) == name:
            return c
    raise AssertionError(f"case not found: {name}")


def test_realcase_all_has_flow_and_opening_margin():
    net = make_net()
    base_hz = 50.0
    out = commission_and_scale(
        net,
        fan_edge_ids=list(FAN_EDGE_IDS),
        full_active_cav_edge_ids=list(CAV_EDGES),
        q_design_by_edge=dict(Q_DESIGN),
        scaling_cases=list(SCALING_CASES),
        fixed_p=dict(FIXED_P),
        d={},
        model=str(REALCASE_DAMPER_MODEL_DEFAULT),
        gamma=float(REALCASE_DAMPER_GAMMA_DEFAULT),
        s_min=float(REALCASE_HZ_MIN_DEFAULT) / base_hz,
        s_max=float(REALCASE_HZ_MAX_DEFAULT) / base_hz,
        theta_center=float(REALCASE_THETA_CENTER_DEFAULT),
        theta_band=float(REALCASE_THETA_BAND_DEFAULT),
        theta_off_deg=float(REALCASE_THETA_OFF_DEFAULT),
        off_damper_u=float(REALCASE_OFF_DAMPER_U_DEFAULT),
        tol_q=0.003,
        eps_under_rel=0.01,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    assert out.res_full.converged
    assert out.full_load_diag in {"ok", "under_static", "over_static"}

    all_case = _get_case(out, "all")
    assert all_case.ok
    assert all_case.res.converged

    ratios = [abs(float(all_case.res.q[eid])) / float(Q_DESIGN[eid]) for eid in CAV_EDGES]
    assert min(ratios) >= 0.985
    assert max(ratios) <= 1.02

    max_theta = max(float(all_case.thetas[eid]) for eid in CAV_EDGES)
    # Keep headroom; full-load should avoid 90deg saturation.
    assert 70.0 <= max_theta <= 85.0

    # Keep full-load speed in a realistic operating band for this real-case setup.
    assert 0.65 <= float(out.speed_full) <= 0.90
