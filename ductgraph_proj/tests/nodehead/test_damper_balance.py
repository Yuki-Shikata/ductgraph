import math
from ductgraph.model import Node, Edge, Network
from ductgraph.control_damper import balance_dampers_to_targets
from ductgraph.solver_nodehead import solve_node_head


def test_balance_two_terminals_to_equal_targets_within_angle_band():
    # 2つの端末（同一条件）に対して、片側にダンパを持たせ
    # 65-85度の範囲で目標風量に“十分近く”合わせられることを確認する。
    #
    # NOTE:
    # 65-85deg はほぼ全開域で、ダンパの効きが弱く分解能が粗い。
    # よって許容誤差は 2e-3 程度（=0.2%）を現実的な基準として採用。

    rho, cd = 1.2, 0.65
    A = 0.03
    Qin = 2.0

    net = Network(
        nodes=[Node(0, "branch"), Node(1, "amb1"), Node(2, "amb2")],
        edges=[
            Edge(id=0, frm=0, to=1, r=0.0, terminal_orifice=(rho, cd, A)),
            Edge(id=1, frm=0, to=2, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        ],
        ref_node=1,
    )

    d = {0: -Qin}
    fixed_p = {1: 0.0, 2: 0.0}

    base = solve_node_head(net, d=d, fixed_p=fixed_p)
    assert base.converged

    targets = {0: Qin / 2.0, 1: Qin / 2.0}

    tol = 2e-3

    thetas, res = balance_dampers_to_targets(
        net,
        targets_q=targets,
        damper_edge_ids=[1],
        theta_center=75.0,
        theta_band=10.0,
        fixed_p=fixed_p,
        d=d,
        model="sin",
        gamma=1.0,
        tol_q=tol,
    )

    assert res.converged
    assert 65.0 <= thetas[1] <= 85.0

    assert math.isclose(res.q[0], Qin / 2.0, rel_tol=0.0, abs_tol=tol)
    assert math.isclose(res.q[1], Qin / 2.0, rel_tol=0.0, abs_tol=tol)
