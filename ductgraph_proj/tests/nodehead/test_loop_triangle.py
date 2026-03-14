import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_single_loop_triangle_two_outlets():
    # 三角形ループ（0-1-2-0）だが、
    # Node1 と Node2 を外気(0Pa)に固定して、境界条件を物理的に成立させる。
    #
    # 目的：
    # - ループがあっても収束する
    # - 連続条件が成立する
    # - “余計な循環”が支配的にならず、分流として解が出る

    Qin = 4.0
    r01 = 1.0
    r12 = 2.0
    r20 = 3.0

    net = Network(
        nodes=[Node(0, "supply"), Node(1, "out1"), Node(2, "out2")],
        edges=[
            Edge(id=0, frm=0, to=1, r=r01),
            Edge(id=1, frm=1, to=2, r=r12),
            Edge(id=2, frm=2, to=0, r=r20),
        ],
        ref_node=1,
    )

    # Node0 から Qin 流出（これまでの規約）
    d = {0: -Qin}

    # Node1, Node2 を外気として 0Pa 固定
    res = solve_node_head(net, d, fixed_p={1: 0.0, 2: 0.0})
    assert res.converged, f"not converged iters={res.iters} res={res.residual_norm}"

    Q01 = res.q[0]
    Q12 = res.q[1]
    Q20 = res.q[2]

    # 連続条件（free node は Node0 のみになるので、ここだけチェックでOK）
    # Node0 行： -Q01 + Q20 = d0 = -Qin
    assert math.isclose((-Q01 + Q20), -Qin, abs_tol=1e-9)

    # 固定圧なので p1=p2=0
    assert math.isclose(res.p[1], 0.0, abs_tol=1e-12)
    assert math.isclose(res.p[2], 0.0, abs_tol=1e-12)

    # “循環が巨大になっていない”ことだけ軽くチェック（ゼロを強制しない）
    # 目安：供給 Qin と同程度のオーダーに収まること
    assert abs(Q01) < 10 * Qin
    assert abs(Q12) < 10 * Qin
    assert abs(Q20) < 10 * Qin
