import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_damper_closing_increases_required_dp_at_fixed_flow():
    # 0 --(damper)--> 1(ambient 0Pa)
    # Qout を固定して、開度 u を下げると必要dpが増えるはず

    Qout = 2.0

    def run(u: float):
        net = Network(
            nodes=[Node(0, "hood"), Node(1, "amb")],
            edges=[
                Edge(id=0, frm=0, to=1, r=1.0, damper_k=4.0, damper_u=u),
            ],
            ref_node=1,
        )
        return solve_node_head(net, d={0: -Qout}, fixed_p={1: 0.0})

    res_open = run(1.0)
    res_half = run(0.5)

    assert res_open.converged
    assert res_half.converged

    assert math.isclose(res_open.q[0], Qout, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(res_half.q[0], Qout, rel_tol=1e-9, abs_tol=1e-12)

    dp_open = abs(res_open.p[0] - res_open.p[1])
    dp_half = abs(res_half.p[0] - res_half.p[1])

    assert dp_half > dp_open
