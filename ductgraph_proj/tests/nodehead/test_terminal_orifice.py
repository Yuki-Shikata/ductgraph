import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def test_terminal_orifice_area_reduces_required_dp_at_fixed_flow():
    # 0 --(terminal/orifice)--> 1(ambient 0Pa)
    #
    # 端末の評価は「同一流量で必要な圧力差がどう変わるか」が本質。
    # Area を大きくすると損失が下がるので、必要 dp = |p0-p1| は小さくなるはず。

    rho = 1.2
    cd = 0.65
    Qout = 2.0  # Node0 から外気へ流出させる（固定流量）

    def run(area: float):
        net = Network(
            nodes=[Node(0, "hood"), Node(1, "amb")],
            edges=[
                Edge(id=0, frm=0, to=1, r=0.0, terminal_orifice=(rho, cd, area)),
            ],
            ref_node=1,
        )
        # これまでの規約：ノードからQ流出させたい -> d[node] = -Q
        res = solve_node_head(net, d={0: -Qout}, fixed_p={1: 0.0})
        return res

    res_small = run(area=0.02)
    res_large = run(area=0.05)

    assert res_small.converged
    assert res_large.converged

    # 流量は指定値になる
    assert math.isclose(res_small.q[0], Qout, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(res_large.q[0], Qout, rel_tol=1e-9, abs_tol=1e-12)

    dp_small = abs(res_small.p[0] - res_small.p[1])
    dp_large = abs(res_large.p[0] - res_large.p[1])

    # 面積を大きくすると必要dpは下がる
    assert dp_large < dp_small
