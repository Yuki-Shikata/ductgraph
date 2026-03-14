import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_two_nodes_one_edge():
    net = Network(
        nodes=[Node(0, "up"), Node(1, "ref")],
        edges=[Edge(id=0, frm=0, to=1, r=2.0)],
        ref_node=1,
    )

    Qset = 3.0
    d = {0: -Qset}

    res = solve_node_head(net, d)

    assert res.converged
    assert math.isclose(res.q[0], Qset, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(res.p[0], 2.0 * Qset**2, rel_tol=1e-6, abs_tol=1e-8)
    assert math.isclose(res.p[1], 0.0, abs_tol=1e-12)
