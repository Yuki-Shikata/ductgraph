import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_ideal_fan_creates_circulation_in_single_loop():
    # ループ：0->1->2->0
    # Edge0(0->1) に理想ファン +H を入れる（h=H）
    # 外部流量 d は無し（全ノードで連続条件 A q = 0）
    #
    # 直列ループなので、全枝で流量は同じ Q になる。
    # 閉路条件（圧力の整合）から：
    #   (r0 + r12 + r20) * Q^2 = H
    # => Q = sqrt(H / r_sum)

    H = 36.0  # Pa（わかりやすい数）
    r0 = 1.0
    r12 = 2.0
    r20 = 6.0
    r_sum = r0 + r12 + r20

    net = Network(
        nodes=[Node(0, "n0"), Node(1, "n1"), Node(2, "n2")],
        edges=[
            Edge(id=0, frm=0, to=1, r=r0, h=H),   # ファン入り
            Edge(id=1, frm=1, to=2, r=r12),
            Edge(id=2, frm=2, to=0, r=r20),
        ],
        ref_node=0,
    )

    # 外部流量なし
    d = {}

    # 圧力基準だけ固定（どれか1点を0Paにしないと圧力が一意に決まらない）
    res = solve_node_head(net, d, fixed_p={0: 0.0})
    assert res.converged, f"not converged iters={res.iters} res={res.residual_norm}"

    Q0 = res.q[0]
    Q1 = res.q[1]
    Q2 = res.q[2]

    # 直列ループなので同一流量になるはず（符号も同じはず）
    assert math.isclose(Q0, Q1, rel_tol=1e-8, abs_tol=1e-10)
    assert math.isclose(Q1, Q2, rel_tol=1e-8, abs_tol=1e-10)

    Q_expected = math.sqrt(H / r_sum)
    assert math.isclose(Q0, Q_expected, rel_tol=1e-6, abs_tol=1e-9)
