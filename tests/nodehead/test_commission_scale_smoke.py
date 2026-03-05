import math

from ductgraph.model import Network, Node, Edge
from ductgraph.commissioning_scale import commission_and_scale


def test_commission_and_scale_smoke_runs_and_reports_saturation():
    # fan -> junction -> 3 terminals (2 easy + 1 hard=maxload)
    rho, cd = 1.2, 0.65
    A_easy = 0.06
    A_hard = 0.03

    a3, a2, a1, a0 = 0.0, -1.0, 0.0, 450.0
    r_fan_duct = 2.0
    r_hard_extra = 10.0

    net = Network(
        nodes=[Node(0, "amb_in"), Node(1, "jct"), Node(2, "amb_e1"), Node(3, "amb_e2"), Node(4, "amb_h")],
        edges=[
            Edge(id=0, frm=0, to=1, r=r_fan_duct, fan_poly=(a3, a2, a1, a0), speed_ratio=1.0),
            Edge(id=1, frm=1, to=2, r=0.0, terminal_orifice=(rho, cd, A_easy), damper_k=8.0, damper_u=1.0),
            Edge(id=2, frm=1, to=3, r=0.0, terminal_orifice=(rho, cd, A_easy), damper_k=8.0, damper_u=1.0),
            Edge(id=3, frm=1, to=4, r=r_hard_extra, terminal_orifice=(rho, cd, A_hard), damper_k=10.0, damper_u=1.0),
        ],
        ref_node=0,
    )

    fixed_p = {0: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

    q_design = {1: 1.0, 2: 1.0, 3: 1.0}
    full_active = [1, 2, 3]

    # scaling: 2台運転に落とす（線形スケール）
    # ※物理的には「出ない」ケースがあり得るので、ここでは “動いてレポートできる” を確認する
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

    assert out.res_full.converged
    assert out.speed_full > 0.0
    assert len(out.cases) == 3
    for c in out.cases:
        assert c.res.converged
        # active edges should exist in achieved map
        for eid in [int(x) for x in c.thetas.keys()]:
            assert eid in c.achieved_q_abs
            assert c.achieved_q_abs[eid] >= 0.0
