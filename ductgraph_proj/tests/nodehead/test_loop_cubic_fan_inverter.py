import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_loop_cubic_fan_inverter_reduces_circulation():
    # Loop: 0->1->2->0
    # Edge0: cubic fan with inverter speed_ratio
    # No external flow => single-loop circulation determined by fan vs losses.
    #
    # Expectation: lowering speed_ratio reduces |Q|.

    r0 = 1.0
    r12 = 2.0
    r20 = 6.0

    # cubic fan
    a3 = 0.2
    a2 = -1.0
    a1 = 0.0
    a0 = 36.0

    def run(speed_ratio: float):
        net = Network(
            nodes=[Node(0, "n0"), Node(1, "n1"), Node(2, "n2")],
            edges=[
                Edge(id=0, frm=0, to=1, r=r0, fan_poly=(a3, a2, a1, a0), speed_ratio=speed_ratio),
                Edge(id=1, frm=1, to=2, r=r12),
                Edge(id=2, frm=2, to=0, r=r20),
            ],
            ref_node=0,
        )
        return solve_node_head(
            net,
            d={},
            fixed_p={0: 0.0},
            fan_q_init=2.0,   # 小流量根側に寄せる
            fan_q_cap=10.0,
        )

    res1 = run(1.0)
    res2 = run(0.8)

    assert res1.converged
    assert res2.converged

    Q1 = res1.q[0]
    Q2 = res2.q[0]

    # ループなので3枝は同一流量（符号は方向）
    assert math.isclose(res1.q[0], res1.q[1], rel_tol=1e-7, abs_tol=1e-10)
    assert math.isclose(res1.q[1], res1.q[2], rel_tol=1e-7, abs_tol=1e-10)
    assert math.isclose(res2.q[0], res2.q[1], rel_tol=1e-7, abs_tol=1e-10)
    assert math.isclose(res2.q[1], res2.q[2], rel_tol=1e-7, abs_tol=1e-10)

    # speed_ratio を下げると循環流の絶対値は下がるはず
    assert abs(Q2) < abs(Q1)
