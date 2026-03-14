import math
from ductgraph.model import Network, Node, Edge
from ductgraph.commissioning import commission_two_stage

def test_commission_tree5_converges_and_meets_design():
    # terminal model
    rho, cd = 1.2, 0.65

    # fan curve (dummy but monotone-ish): dp = a0 + a2*Q^2
    # NOTE: fan_poly is cubic (a3,a2,a1,a0)
    a3, a2, a1, a0 = 0.0, -120.0, 0.0, 1300.0

    # design flow per terminal (m3/s)
    Qd = 0.40

    # Damper throttle range: 0..75deg (throttle only)
    other_theta_min = 0.0
    other_theta_max = 75.0

    # "tree" duct resistances (Pa/(m3/s)^2-ish in your simplified model)
    r_fan_duct = 1.0
    r_trunk = 2.0
    r_branchA = 3.0
    r_branchB = 5.0

    # make one terminal harder (extra resistance) so it becomes maxload
    r_hard_extra = 12.0

    # terminal orifices (area) - same for all to keep focus on duct resistances
    A = 0.05

    # Nodes:
    # 0 amb_in -> 1 fan_out -> 2 trunk_jct -> 3 jctA -> terminals (5,6)
    #                                \-> 4 jctB -> terminals (7,8,9)
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

        # trunk
        Edge(id=10, frm=1, to=2, r=r_trunk),

        # branches
        Edge(id=11, frm=2, to=3, r=r_branchA),
        Edge(id=12, frm=2, to=4, r=r_branchB),

        # terminals on branch A (2 terminals)
        Edge(id=1, frm=3, to=5, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=2, frm=3, to=6, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),

        # terminals on branch B (3 terminals; last is harder)
        Edge(id=3, frm=4, to=7, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=4, frm=4, to=8, r=0.0, terminal_orifice=(rho, cd, A), damper_k=8.0, damper_u=1.0),
        Edge(id=5, frm=4, to=9, r=r_hard_extra, terminal_orifice=(rho, cd, A), damper_k=10.0, damper_u=1.0),
    ]

    net = Network(nodes=nodes, edges=edges, ref_node=0)

    # Boundary conditions: all terminals are to ambient (0 Pa), inlet also 0 Pa.
    fixed_p = {0: 0.0, 5: 0.0, 6: 0.0, 7: 0.0, 8: 0.0, 9: 0.0}

    # We want maxload around 75deg. In this constructed network, edge 5 tends to be maxload.
    out = commission_two_stage(
        net,
        fan_edge_ids=[0],
        maxload_edge_id=5,                  # "hard" terminal
        other_damper_edge_ids=[1,2,3,4],     # other terminals
        q_design=Qd,
        theta_center=75.0,
        theta_band=10.0,
        s_min=0.2,
        s_max=2.5,
        other_theta_min=other_theta_min,
        other_theta_max=other_theta_max,
        rounds=60,
        tol_q=0.03 * Qd,                     # 3% (設備評価tol)
        fixed_p=fixed_p,
        d={},
        model="sin",
        gamma=1.0,
        fan_q_init=2.0,
        fan_q_cap=80.0,
    )

    assert out.res.converged, out.msg
    assert out.ok, out.msg

    # Must NOT be under design (undershoot NG) for every terminal
    for eid in [1,2,3,4,5]:
        assert out.res.q[eid] >= (Qd * (1.0 - 0.0002)), (eid, out.res.q[eid], Qd)

    # maxload is the most-open terminal in final result
    th_max = max(out.thetas[eid] for eid in [1,2,3,4,5])
    assert math.isclose(out.thetas[5], th_max, rel_tol=0.0, abs_tol=1e-9), out.thetas
