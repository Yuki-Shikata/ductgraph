# ============================================================
# Test spec: "insufficient fan" (到達不能) の定義
#
# このテストが確認したいこと：
#   commissioning が回転数（speed_ratio）を上げても設計風量に到達できない場合、
#   それを「到達不能」として NG にできること。
#
# 合格条件（= 到達不能を正しく検出できた）：
#   (1) out.ok == False
#       - 最終判定として「全端末で設計風量以上」を満たせない（不足NG）
#   (2) out.speed_ratio が S_MAX（上限）に張り付く
#       - これ以上回転数を上げられず、物理的に到達不能である根拠
#
# 注意：
#   - 風量の許容幅 tol_q(例:3%) は「合否」ではなく、収束/警告の目安に限定する。
#   - 本テストは「性能曲線探し」ではなく、到達不能の判定ロジック検証が目的。
# ============================================================

import math
from ductgraph.model import Network, Node, Edge
from ductgraph.commissioning import commission_two_stage

def test_commission_tree5_insufficient_fan_is_ng():
    rho, cd = 1.2, 0.65

    # fan curve (dummy): dp = a0 + a2*Q^2  (fan_poly is cubic a3,a2,a1,a0)
    a3, a2, a1, a0 = 0.0, -120.0, 0.0, 20.0  # VERY weak fan: must be insufficient  # weaker fan: should be insufficient even at s_max

    Qd = 0.40
    other_theta_min = 0.0
    other_theta_max = 75.0

    r_fan_duct = 1.0
    r_trunk = 2.0
    r_branchA = 3.0
    r_branchB = 5.0
    r_hard_extra = 12.0

    A = 0.05

    nodes = [
        Node(0, "amb_in"),
        Node(1, "fan_out"),
        Node(2, "trunk_jct"),
        Node(3, "jctA"),
        Node(4, "jctB"),
        Node(5, "tA1"),
        Node(6, "tA2"),
        Node(7, "tB1"),
        Node(8, "tB2"),
        Node(9, "tB3_hard"),
    ]

    edges = [
        # fan
        Edge(id=0, frm=0, to=1, r=r_fan_duct, fan_poly=(a3, a2, a1, a0), speed_ratio=1.0),

        # trunk / branches
        Edge(id=10, frm=1, to=2, r=r_trunk),
        Edge(id=11, frm=2, to=3, r=r_branchA),
        Edge(id=12, frm=2, to=4, r=r_branchB),

        # terminals
        Edge(id=1, frm=3, to=5, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=2, frm=3, to=6, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=3, frm=4, to=7, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=4, frm=4, to=8, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=5, frm=4, to=9, r=r_hard_extra, terminal_orifice=(rho, cd, A), damper_k=10.0, damper_u=1.0),
    ]

    net = Network(nodes=nodes, edges=edges, ref_node=0)
    fixed_p = {0: 0.0, 5: 0.0, 6: 0.0, 7: 0.0, 8: 0.0, 9: 0.0}

    S_MAX = 1.0  # speed upper limit used in this test
    out = commission_two_stage(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=5,
        other_damper_edge_ids=[1, 2, 3, 4],
        q_design=Qd,
        theta_center=75.0,
        theta_band=10.0,
        s_min=0.2,
        s_max=S_MAX,
        other_theta_min=other_theta_min,
        other_theta_max=other_theta_max,
        rounds=60,
        tol_q=0.03 * Qd,
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    assert out.res.converged, out.msg
    # 不足はNG（仕様）
    # 期待：ファン能力不足なら commissioning は NG を返す
    assert out.res.converged, out.msg
    assert (not out.ok), out.msg

    # さらに「回転数を上げ切った（上限に張り付いた）」ことを確認（到達不能の根拠）
    # ※ speed_ratio は CommissionResult.speed_ratio
    assert abs(out.speed_ratio - S_MAX) < 1e-9, (out.speed_ratio, s_max, out.msg)

