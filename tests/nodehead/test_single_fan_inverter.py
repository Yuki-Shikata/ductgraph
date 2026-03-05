import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_single_cubic_fan_with_inverter():
    a3, a2, a1, a0 = 0.1, -0.5, 0.0, 25.0
    r = 2.0
    Qout = 2.0  # Node0から流出

    def run(speed_ratio: float):
        net = Network(
            nodes=[Node(0), Node(1)],
            edges=[
                Edge(
                    id=0,
                    frm=0,
                    to=1,
                    r=r,
                    fan_poly=(a3, a2, a1, a0),
                    speed_ratio=speed_ratio,
                )
            ],
            ref_node=1,
        )
        return solve_node_head(
            net,
            d={0: -Qout},
            fixed_p={1: 0.0},
            fan_q_init=Qout,
            fan_q_cap=10.0,
        )

    res1 = run(1.0)
    res2 = run(0.8)

    assert res1.converged
    assert res2.converged

    assert math.isclose(res1.q[0], Qout, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(res2.q[0], Qout, rel_tol=1e-9, abs_tol=1e-12)

    p01 = abs(res1.p[0] - res1.p[1])
    p02 = abs(res2.p[0] - res2.p[1])

    # この符号規約では dp = rQ^2 - h なので
    # 回転数↓ → h↓ → dp が 0に近づく → |dp| は小さくなる
    assert p02 < p01
