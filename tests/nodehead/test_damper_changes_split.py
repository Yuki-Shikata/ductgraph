import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_damper_reduces_branch_flow_and_increases_other():
    # Node0 から合計Qin流出。2枝は同じ端末だが、片側だけダンパを絞る。
    # -> 絞った枝の流量は減り、もう片方は増える。

    rho, cd = 1.2, 0.65
    A = 0.03
    Qin = 2.0

    def run(u_branch2: float):
        net = Network(
            nodes=[Node(0, "branch"), Node(1, "amb1"), Node(2, "amb2")],
            edges=[
                # branch1: no damper
                Edge(id=0, frm=0, to=1, r=0.0, terminal_orifice=(rho, cd, A)),
                # branch2: with damper
                Edge(id=1, frm=0, to=2, r=0.0, terminal_orifice=(rho, cd, A), damper_k=5.0, damper_u=u_branch2),
            ],
            ref_node=1,
        )
        return solve_node_head(net, d={0: -Qin}, fixed_p={1: 0.0, 2: 0.0})

    res_open = run(1.0)   # damper open
    res_close = run(0.5)  # damper half-close

    assert res_open.converged
    assert res_close.converged

    Q1_open, Q2_open = res_open.q[0], res_open.q[1]
    Q1_close, Q2_close = res_close.q[0], res_close.q[1]

    # continuity
    assert math.isclose(Q1_open + Q2_open, Qin, rel_tol=1e-8, abs_tol=1e-10)
    assert math.isclose(Q1_close + Q2_close, Qin, rel_tol=1e-8, abs_tol=1e-10)

    # closing damper reduces branch2
    assert Q2_close < Q2_open
    # and increases branch1
    assert Q1_close > Q1_open
