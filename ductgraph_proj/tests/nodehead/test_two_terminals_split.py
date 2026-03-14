import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_two_terminals_flow_splits_by_area():
    # Node0 (branch) -> Node1 (amb 0Pa) via Terminal A
    #                -> Node2 (amb 0Pa) via Terminal B
    #
    # Node1/Node2 はどちらも外気(0Pa)固定とみなせるので、
    # 分岐点Node0から見た圧力差は2枝で同一になる。
    #
    # Terminal(orifice) model:
    #   Δp = (rho/2)*(Q/(Cd*A))^2
    # => for same Δp: Q ∝ Cd*A
    #
    # よって Cd同じなら Q比は面積比にほぼ一致。

    rho = 1.2
    cd = 0.65

    A1 = 0.02
    A2 = 0.05

    Qin = 2.0  # Node0 から合計で流出させる

    net = Network(
        nodes=[Node(0, "branch"), Node(1, "amb1"), Node(2, "amb2")],
        edges=[
            Edge(id=0, frm=0, to=1, r=0.0, terminal_orifice=(rho, cd, A1)),
            Edge(id=1, frm=0, to=2, r=0.0, terminal_orifice=(rho, cd, A2)),
        ],
        ref_node=1,
    )

    # Node0から Qin 流出
    res = solve_node_head(net, d={0: -Qin}, fixed_p={1: 0.0, 2: 0.0})
    assert res.converged

    Q1 = res.q[0]
    Q2 = res.q[1]

    # 連続条件：Q1 + Q2 = Qin
    assert math.isclose(Q1 + Q2, Qin, rel_tol=1e-8, abs_tol=1e-10)

    # 面積の大きい方が流量が大きい
    assert Q2 > Q1

    # 理論比（同一dpなら Q比 ≈ A比）
    ratio = Q1 / Q2
    expected = A1 / A2
    assert math.isclose(ratio, expected, rel_tol=5e-2)  # 5%程度の誤差を許容
