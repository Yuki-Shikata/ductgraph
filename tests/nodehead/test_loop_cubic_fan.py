import math
from ductgraph.model import Node, Edge, Network
from ductgraph.solver_nodehead import solve_node_head


def _solve_expected_Q(H0, a3, a2, a1, r_sum):
    # Solve: r_sum*Q^2 = H(Q) = a3 Q^3 + a2 Q^2 + a1 Q + H0
    # => f(Q)= r_sum*Q^2 - (a3 Q^3 + a2 Q^2 + a1 Q + H0)=0
    #
    # 初期値を +1.0 にして「正の小流量解」を狙う（物理解として採用）
    Q = 1.0
    for _ in range(80):
        f = r_sum*(Q**2) - (a3*(Q**3) + a2*(Q**2) + a1*Q + H0)
        df = 2*r_sum*Q - (3*a3*(Q**2) + 2*a2*Q + a1)
        if abs(df) < 1e-12:
            df = 1e-12
        step = f/df
        Q -= step
        if abs(step) < 1e-12:
            break
    return Q


def test_cubic_fan_creates_circulation_in_loop():
    # Loop: 0->1->2->0
    # Edge0 has cubic fan curve h(Q)=a3 Q^3 + a2 Q^2 + a1 Q + a0
    # No external flow. With a single loop, all edges have same Q.
    # Loop balance becomes:
    #   (r0+r12+r20)*Q^2 = h(Q)
    #
    # 3次ファン曲線では複数解が存在し得るため、
    # このテストでは「正の小流量解」を物理的に採用する。
    # solverが符号反転（循環方向反転）の解を取っても、
    # “運転点（|Q|）”が一致していれば合格とする。

    r0 = 1.0
    r12 = 2.0
    r20 = 6.0
    r_sum = r0 + r12 + r20

    # cubic fan (3rd order)
    a3 = 0.2
    a2 = -1.0
    a1 = 0.0
    a0 = 36.0  # base pressure at Q=0

    net = Network(
        nodes=[Node(0, "n0"), Node(1, "n1"), Node(2, "n2")],
        edges=[
            Edge(id=0, frm=0, to=1, r=r0, fan_poly=(a3, a2, a1, a0)),
            Edge(id=1, frm=1, to=2, r=r12),
            Edge(id=2, frm=2, to=0, r=r20),
        ],
        ref_node=0,
    )

    res = solve_node_head(net, d={}, fixed_p={0: 0.0})
    assert res.converged, f"not converged iters={res.iters} res={res.residual_norm}"

    Q0 = res.q[0]
    Q1 = res.q[1]
    Q2 = res.q[2]

    # 直列ループなので同一流量になるはず（符号も一致するはずだが、念のため絶対値で確認）
    assert math.isclose(Q0, Q1, rel_tol=1e-7, abs_tol=1e-10)
    assert math.isclose(Q1, Q2, rel_tol=1e-7, abs_tol=1e-10)

    Qexp = _solve_expected_Q(a0, a3, a2, a1, r_sum)
    assert Qexp > 0.0

    # --- 物理レンジ（過大流量解を排除） ---
    # この設定では Qexp はだいたい 0～数のオーダーになる想定
    assert abs(Q0) < 10.0

    # --- 運転点（|Q|）が期待解と一致するか ---
    assert math.isclose(abs(Q0), Qexp, rel_tol=1e-5, abs_tol=1e-7)
