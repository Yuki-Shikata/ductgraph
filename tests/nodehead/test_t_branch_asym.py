import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_asymmetric_t_branch_with_extra_resistance():
    # 非対称T分岐：
    # - Node0 から 2枝に分流
    # - 両終端(Node1, Node2)は外気として 0Pa 固定
    # - 片側（Node2側）に追加抵抗を直列に入れる（ダンパ・長ダクト・局部損失の代用）

    Qin = 6.0

    r1 = 1.0
    r2 = 1.0
    r_add = 8.0  # 追加抵抗（強めにして分流差を分かりやすくする）
    r2_eq = r2 + r_add

    # ネットワーク：
    # Edge0: 0 -> 1 (r1)
    # Edge1: 0 -> 2a (r2)
    # Edge2: 2a -> 2 (r_add)
    #
    # (2a は「追加抵抗を表現するためだけの中間ノード」)
    net = Network(
        nodes=[
            Node(0, "branch"),
            Node(1, "out1"),
            Node(20, "mid2"),  # 中間ノード（仮のID）
            Node(2, "out2"),
        ],
        edges=[
            Edge(id=0, frm=0, to=1, r=r1),
            Edge(id=1, frm=0, to=20, r=r2),
            Edge(id=2, frm=20, to=2, r=r_add),
        ],
        ref_node=1,
    )

    # Node0 から Qin 流出
    d = {0: -Qin}

    # 両終端を 0Pa 固定（外気）
    res = solve_node_head(net, d, fixed_p={1: 0.0, 2: 0.0})
    assert res.converged, f"not converged iters={res.iters} res={res.residual_norm}"

    Q1 = res.q[0]     # 0->1
    Q2 = res.q[1]     # 0->20  （= 20->2 と同じになるはず）

    # 連続条件（Node0）
    assert math.isclose(Q1 + Q2, Qin, rel_tol=1e-8, abs_tol=1e-10)

    # 直列なので Node2側の流量は Edge1 と Edge2 で一致するはず
    assert math.isclose(res.q[1], res.q[2], rel_tol=1e-9, abs_tol=1e-12)

    # 期待される分流比： Q1/Q2 = sqrt(r2_eq/r1)
    expected_ratio = math.sqrt(r2_eq / r1)
    assert math.isclose(Q1 / Q2, expected_ratio, rel_tol=1e-6, abs_tol=1e-10)

    # 参考：分岐点圧力一致（Node0圧力）
    p0 = res.p[0]
    assert math.isclose(p0, r1 * (Q1**2), rel_tol=1e-6, abs_tol=1e-8)
    assert math.isclose(p0, r2_eq * (Q2**2), rel_tol=1e-6, abs_tol=1e-8)
