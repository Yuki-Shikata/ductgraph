import math

from ductgraph.model import Node, Edge, Network
from ductgraph.control_speed import tune_speed_for_maxload, with_speed_ratio
from ductgraph.control_damper import with_damper_angle
from ductgraph.solver_nodehead import solve_node_head


def test_tune_speed_sets_maxload_at_design_flow_with_theta_75():
    # Node0: 外気(吸込み) 0Pa固定（基準圧）
    # Fan:   0 -> 1
    # 端末:  1 -> 2 (easy), 1 -> 3 (max-load with damper)
    # Node2, Node3: 外気(排気先)として 0Pa固定（簡単化）
    #
    # 目的：
    #   最大負荷ルート(edge id=2)のダンパを theta=75deg に固定し、
    #   VFD(s) を調整して Q_design を満たせることを確認する。

    rho, cd = 1.2, 0.65
    A_easy = 0.05
    A_hard = 0.03

    # fan curve (simple) + duct resistance
    # a0 を大きくして「到達可能」なテスト条件にする
    a3, a2, a1, a0 = 0.0, -1.0, 0.0, 400.0
    r_fan_duct = 2.0

    # max-load branch extra loss + damper
    r_hard_extra = 6.0

    Q_design = 1.0

    net = Network(
        nodes=[
            Node(0, "amb_in"),
            Node(1, "jct"),
            Node(2, "amb_easy"),
            Node(3, "amb_hard"),
        ],
        edges=[
            Edge(id=0, frm=0, to=1, r=r_fan_duct, fan_poly=(a3, a2, a1, a0), speed_ratio=1.0),
            Edge(id=1, frm=1, to=2, r=0.0, terminal_orifice=(rho, cd, A_easy)),
            Edge(id=2, frm=1, to=3, r=r_hard_extra, terminal_orifice=(rho, cd, A_hard), damper_k=10.0, damper_u=1.0),
        ],
        ref_node=0,
    )

    fixed_p = {0: 0.0, 2: 0.0, 3: 0.0}

    best_s, (q65, q85, res) = tune_speed_for_maxload(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=2,
        q_design=Q_design,
        theta_center=75.0,
        theta_band=10.0,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        s_min=0.2,
        s_max=3.0,      # 余裕を持たせる
        tol_q=2e-3,
        fan_q_init=2.0,
        fan_q_cap=50.0,
    )

    assert res.converged
    assert 0.2 <= best_s <= 3.0

    # check at theta=75 explicitly
    net_s = with_speed_ratio(net, best_s, fan_edge_ids=[0])
    net_st = with_damper_angle(net_s, {2: 75.0}, model="sin", gamma=1.0)
    res75 = solve_node_head(net_st, d={}, fixed_p=fixed_p, fan_q_init=2.0, fan_q_cap=50.0)

    assert res75.converged
    assert math.isclose(res75.q[2], Q_design, rel_tol=0.0, abs_tol=2e-3)

    # authority check: 65deg should reduce flow vs 85deg
    assert q65 < q85
    assert (q65 - 1e-6) <= Q_design <= (q85 + 1e-6)
