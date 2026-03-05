import math

from ductgraph.model import Node, Edge, Network
from ductgraph.commissioning import commission_two_stage


def test_commission_two_stage_all_terminals_meet_design_with_maxload_at_75():
    rho, cd = 1.2, 0.65

    A_easy = 0.06
    A_hard = 0.03

    a3, a2, a1, a0 = 0.0, -1.0, 0.0, 450.0
    r_fan_duct = 2.0

    r_hard_extra = 10.0
    Q_design = 1.0

    net = Network(
        nodes=[
            Node(0, "amb_in"),
            Node(1, "jct"),
            Node(2, "amb_e1"),
            Node(3, "amb_e2"),
            Node(4, "amb_hard"),
        ],
        edges=[
            # fan edge
            Edge(id=0, frm=0, to=1, r=r_fan_duct, fan_poly=(a3, a2, a1, a0), speed_ratio=1.0),
            # easy terminals with dampers
            Edge(id=1, frm=1, to=2, r=0.0, terminal_orifice=(rho, cd, A_easy), damper_k=8.0, damper_u=1.0),
            Edge(id=2, frm=1, to=3, r=0.0, terminal_orifice=(rho, cd, A_easy), damper_k=8.0, damper_u=1.0),
            # hard terminal (maxload)
            Edge(id=3, frm=1, to=4, r=r_hard_extra, terminal_orifice=(rho, cd, A_hard), damper_k=10.0, damper_u=1.0),
        ],
        ref_node=0,
    )

    fixed_p = {0: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

    out = commission_two_stage(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=3,
        other_damper_edge_ids=[1, 2],
        q_design=Q_design,
        theta_center=75.0,
        theta_band=10.0,
        s_min=0.2,
        s_max=3.0,
        other_theta_min=0.0,
        other_theta_max=75.0,   # <=75 (throttle only)
        rounds=40,              # ★ここを増やす（干渉で収束が遅いことがある）
        tol_q=3e-3,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    assert out.res.converged
    assert out.ok, out.msg

    # maxload damper fixed at 75
    assert math.isclose(out.thetas[3], 75.0, rel_tol=0.0, abs_tol=1e-12)

    # others must be <=75
    assert 0.0 <= out.thetas[1] <= 75.0
    assert 0.0 <= out.thetas[2] <= 75.0

    # all terminals meet design
    assert math.isclose(out.res.q[1], Q_design, rel_tol=0.0, abs_tol=3e-3)
    assert math.isclose(out.res.q[2], Q_design, rel_tol=0.0, abs_tol=3e-3)
    assert math.isclose(out.res.q[3], Q_design, rel_tol=0.0, abs_tol=3e-3)
