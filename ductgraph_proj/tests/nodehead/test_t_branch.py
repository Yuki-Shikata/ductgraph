import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_t_branch_flow_split():
    r1 = 1.0
    r2 = 4.0
    Qin = 5.0

    net = Network(
        nodes=[
            Node(0, "branch"),
            Node(1, "out1"),
            Node(2, "out2"),
        ],
        edges=[
            Edge(id=0, frm=0, to=1, r=r1),
            Edge(id=1, frm=0, to=2, r=r2),
        ],
        ref_node=1,
    )

    d = {0: -Qin}

    # Node1, Node2 を外気として 0Pa 固定
    res = solve_node_head(net, d, fixed_p={1: 0.0, 2: 0.0})
    assert res.converged, f"not converged iters={res.iters} res={res.residual_norm}"

    Q1 = res.q[0]
    Q2 = res.q[1]

    # 連続条件
    assert math.isclose(Q1 + Q2, Qin, rel_tol=1e-8, abs_tol=1e-10)

    # 分流比
    expected_ratio = math.sqrt(r2 / r1)
    assert math.isclose(Q1 / Q2, expected_ratio, rel_tol=1e-6, abs_tol=1e-10)

    # 分岐点圧力一致
    p0 = res.p[0]
    assert math.isclose(p0, r1 * (Q1**2), rel_tol=1e-6, abs_tol=1e-8)
    assert math.isclose(p0, r2 * (Q2**2), rel_tol=1e-6, abs_tol=1e-8)
