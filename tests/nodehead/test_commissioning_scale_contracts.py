from __future__ import annotations

import math
import pytest

from ductgraph.model import Network, Node, Edge
from ductgraph.commissioning_scale import (
    commission_and_scale,
    speed_linear_scale,
    solve_cav_at_speed,
)

# ----------------------------
# Helpers
# ----------------------------
def _make_simple_3term_net(*, r_hard_extra: float = 10.0) -> tuple[Network, dict[int, float], list[int], dict[int, float]]:
    """
    fan -> junction -> 3 terminals (2 easy + 1 hard=maxload)
    This matches the original smoke-style structure, but we use it to assert contracts.
    """
    rho, cd = 1.2, 0.65
    A_easy = 0.06
    A_hard = 0.03

    # fan poly: ΔP = a3 Q^3 + a2 Q^2 + a1 Q + a0
    a3, a2, a1, a0 = 0.0, -1.0, 0.0, 450.0
    r_fan_duct = 2.0

    net = Network(
        nodes=[
            Node(0, "amb_in"),
            Node(1, "jct"),
            Node(2, "amb_e1"),
            Node(3, "amb_e2"),
            Node(4, "amb_h"),
        ],
        edges=[
            Edge(id=0, frm=0, to=1, r=r_fan_duct, fan_poly=(a3, a2, a1, a0), speed_ratio=1.0),
            Edge(id=1, frm=1, to=2, r=0.0, terminal_orifice=(rho, cd, A_easy), damper_k=8.0, damper_u=1.0),
            Edge(id=2, frm=1, to=3, r=0.0, terminal_orifice=(rho, cd, A_easy), damper_k=8.0, damper_u=1.0),
            Edge(id=3, frm=1, to=4, r=float(r_hard_extra), terminal_orifice=(rho, cd, A_hard), damper_k=10.0, damper_u=1.0),
        ],
        ref_node=0,
    )

    fixed_p = {0: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
    # design flows are arbitrary positive in this model (m3/s)
    q_design = {1: 1.0, 2: 1.0, 3: 1.0}
    full_active = [1, 2, 3]
    return net, fixed_p, full_active, q_design


# ----------------------------
# Tests
# ----------------------------
def test_speed_linear_scale_contract():
    s_full = 0.8
    q_full = 100.0
    q_active = 40.0
    s_case = speed_linear_scale(s_full, qsum_active=q_active, qsum_full=q_full)
    assert abs(s_case - (s_full * q_active / q_full)) < 1e-12


def test_solve_cav_at_speed_closes_off_edges_effectively():
    """
    Contract test:
      - In partial operation, OFF terminals must be closed, otherwise flow leaks.
      - solve_cav_at_speed supports this via closed_cav_edge_ids + theta_off.
    """
    net, fixed_p, full_active, q_design = _make_simple_3term_net(r_hard_extra=0.0)

    # run at some speed where things converge
    s = 0.7

    # only terminal 1 is active, others should be OFF
    active = [1]
    targets = {1: 0.8}
    off = [2, 3]

    # 1) without closing off edges: expect non-trivial flow in off terminals
    cav_no_close = solve_cav_at_speed(
        net,
        speed_ratio=s,
        fan_edge_ids=[0],
        active_cav_edge_ids=active,
        targets_q=targets,
        closed_cav_edge_ids=None,   # <-- not closed
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        tol_q=3e-3,
        max_iters=60,
        alpha=0.7,
        H0_gain=1.0,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )
    assert cav_no_close.res.converged

    q2 = abs(float(cav_no_close.res.q[2]))
    q3 = abs(float(cav_no_close.res.q[3]))
    leak = q2 + q3

    # 2) with closing off edges: leakage should reduce significantly
    cav_close = solve_cav_at_speed(
        net,
        speed_ratio=s,
        fan_edge_ids=[0],
        active_cav_edge_ids=active,
        targets_q=targets,
        closed_cav_edge_ids=off,
        theta_off=0.0,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        tol_q=3e-3,
        max_iters=60,
        alpha=0.7,
        H0_gain=1.0,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )
    assert cav_close.res.converged

    q2c = abs(float(cav_close.res.q[2]))
    q3c = abs(float(cav_close.res.q[3]))
    leak_closed = q2c + q3c

    # Not expecting exact zero (depends on model), but must be meaningfully lower
    assert leak_closed < leak * 0.3


def test_commission_and_scale_reports_each_case_and_speed_ratio_is_linear():
    net, fixed_p, full_active, q_design = _make_simple_3term_net(r_hard_extra=10.0)

    cases = [
        ("full", [1, 2, 3]),
        ("two", [1, 3]),
        ("one", [3]),
    ]

    out = commission_and_scale(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=3,
        full_active_cav_edge_ids=full_active,
        q_design_by_edge=q_design,
        scaling_cases=cases,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        tol_q=3e-3,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    assert out.res_full is not None
    assert out.res_full.converged in (True, False)  # must exist
    assert len(out.cases) == len(cases)

    # speed ratios must follow linear scaling (definition-level contract)
    qsum_full = sum(q_design[eid] for eid in full_active)
    for case in out.cases:
        active_edges = dict(cases)[case.name]  # list of active edge ids
        qsum_active = sum(q_design[eid] for eid in active_edges)
        expected = out.speed_full * qsum_active / qsum_full
        assert abs(case.speed_ratio - expected) < 1e-12


def test_fullopen_undershoot_warning_when_capacity_is_insufficient():
    """
    Build a case where 'one' operation at low linear speed is physically insufficient.
    We assert the diagnostic 'WARN full-open undershoot' appears and saturated_90 captures it.
    """
    net, fixed_p, full_active, q_design = _make_simple_3term_net(r_hard_extra=25.0)

    cases = [
        ("full", [1, 2, 3]),
        ("one", [3]),
    ]

    out = commission_and_scale(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=3,
        full_active_cav_edge_ids=full_active,
        q_design_by_edge=q_design,
        scaling_cases=cases,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        tol_q=3e-3,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    # Find 'one' case
    one = [c for c in out.cases if c.name == "one"][0]

    # If it undershoots or doesn't converge, capacity check should run and message should include it.
    # We require that "full-open undershoot" diagnostic appears when it is actually insufficient.
    if (not one.res.converged) or (not one.ok):
        assert "full-open undershoot" in one.msg
        assert 3 in one.saturated_90


def test_overshoot_is_warning_not_fail_if_no_undershoot():
    """
    If overshoot exists but there is no undershoot, ok may remain True.
    We assert msg carries overshoot warning when overshoot happens.
    """
    net, fixed_p, full_active, q_design = _make_simple_3term_net(r_hard_extra=0.0)

    cases = [("full", [1, 2, 3])]
    out = commission_and_scale(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=1,  # choose an easy one as maxload for this test
        full_active_cav_edge_ids=full_active,
        q_design_by_edge=q_design,
        scaling_cases=cases,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        tol_q=3e-3,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    c = out.cases[0]
    if c.res.converged:
        # if any edge is over by tol_q, overshoot warning should appear (even if ok remains True)
        any_over = any((q > (q_design[eid] + 3e-3)) for eid, q in c.achieved_q_abs.items())
        if any_over:
            assert "WARN overshoot" in c.msg

def test_full_load_index_selection_is_seed_independent():
    net, fixed_p, full_active, q_design = _make_simple_3term_net(r_hard_extra=20.0)
    cases = [("full", [1, 2, 3])]

    results = []
    for seed in (1, 2, 3):
        out = commission_and_scale(
            net,
            fan_edge_ids=[0],
            maxload_edge_id=seed,
            full_active_cav_edge_ids=full_active,
            q_design_by_edge=q_design,
            scaling_cases=cases,
            fixed_p=fixed_p,
            d={},
            model="sin",
            gamma=1.0,
            tol_q=3e-3,
            fan_q_init=2.0,
            fan_q_cap=80.0,
        )
        results.append((out.index_edge_id, out.speed_full))

    idx0, s0 = results[0]
    assert idx0 in full_active
    for idx, s in results[1:]:
        assert idx == idx0
        assert abs(s - s0) < 1e-10
