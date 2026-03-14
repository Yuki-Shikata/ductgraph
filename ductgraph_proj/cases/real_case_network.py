from __future__ import annotations

from typing import Dict, List

from ductgraph.control_damper import angle_to_u
from ductgraph.ductloss import (
    LocalK,
    Straight,
    r_from_const_dp_at_design,
    r_from_segments_design_point,
    tee_K_from_ratio,
)
from ductgraph.model import Edge, Network, Node

from .real_case_constants import (
    AIR_NU,
    AIR_RHO,
    CAV_EDGES,
    DAMPER_TARGET_DP_PA,
    DAMPER_TARGET_THETA_DEG,
    DEVICE_DP_DESIGN,
    DUCT_ROUGHNESS_M,
    D_BRANCH,
    D_TRUNK,
    EDGE_A,
    EDGE_C,
    EDGE_D,
    EDGE_FAN,
    EDGE_TRUNK_T1,
    EDGE_TRUNK_T2,
    FIXED_P,
    K_BEND_90,
    K_REDUCER,
    NODE_AMB,
    NODE_FAN_OUT,
    NODE_ROOM_A,
    NODE_ROOM_C,
    NODE_ROOM_D,
    NODE_T1,
    NODE_T2,
    SETPOINTS_CMH,
    TEE_K_BRANCH_TABLE,
    TEE_K_STRAIGHT_TABLE,
)


from .real_case_defaults import (
    REALCASE_DAMPER_GAMMA_DEFAULT,
    REALCASE_DAMPER_MODEL_DEFAULT,
    REALCASE_FAN_POLY_DEFAULT,
)


def cmh_to_m3s(q_cmh: float) -> float:
    return float(q_cmh) / 3600.0


def build_q_design() -> Dict[int, float]:
    return {
        EDGE_A: cmh_to_m3s(SETPOINTS_CMH["a"]),
        EDGE_C: cmh_to_m3s(SETPOINTS_CMH["c"]),
        EDGE_D: cmh_to_m3s(SETPOINTS_CMH["d"]),
    }


def _r_duct(*, q_design: float, straight_runs: List[Straight], locals_k: List[LocalK]) -> float:
    return float(
        r_from_segments_design_point(
            Q_m3s=q_design,
            rho=AIR_RHO,
            nu_m2s=AIR_NU,
            roughness_m=DUCT_ROUGHNESS_M,
            straight_runs=straight_runs,
            locals_K=locals_k,
            safety_factor=1.0,
        )
    )


def _compute_edge_r(q_design_by_edge: Dict[int, float]) -> Dict[int, float]:
    q_a = float(q_design_by_edge[EDGE_A])
    q_c = float(q_design_by_edge[EDGE_C])
    q_d = float(q_design_by_edge[EDGE_D])
    q_t2 = q_c + q_d
    q_t1 = q_a + q_t2

    # Tee at T1: parent=q_t1, branch=a, straight=t2-trunk
    _, k_t1_branch_a, k_t1_straight_to_t2 = tee_K_from_ratio(
        q_branch=q_a,
        q_straight=q_t2,
        table_branch=TEE_K_BRANCH_TABLE,
        table_straight=TEE_K_STRAIGHT_TABLE,
    )

    # Tee at T2: parent=q_t2, branches c/d (equal design flow here)
    _, k_t2_branch_c, _ = tee_K_from_ratio(
        q_branch=q_c,
        q_straight=q_d,
        table_branch=TEE_K_BRANCH_TABLE,
        table_straight=TEE_K_STRAIGHT_TABLE,
    )
    _, k_t2_branch_d, _ = tee_K_from_ratio(
        q_branch=q_d,
        q_straight=q_c,
        table_branch=TEE_K_BRANCH_TABLE,
        table_straight=TEE_K_STRAIGHT_TABLE,
    )

    # Fan -> T1 shared trunk (from provided segment list)
    trunk_shared_straights = [2.150, 3.740, 3.000, 1.670, 1.050, 1.420]
    trunk_shared_locals = [1.0, 0.15, 0.30] + [K_BEND_90] * 5

    r_trunk_t1 = _r_duct(
        q_design=q_t1,
        straight_runs=[Straight(L_m=L, D_m=D_TRUNK) for L in trunk_shared_straights],
        locals_k=[LocalK(K, D_m=D_TRUNK) for K in trunk_shared_locals],
    )

    # T1 -> T2 trunk (b trunk from provided list) + T1 tee straight loss
    r_trunk_t2 = _r_duct(
        q_design=q_t2,
        straight_runs=[
            Straight(L_m=1.420, D_m=D_TRUNK),
            Straight(L_m=5.284, D_m=D_TRUNK),
        ],
        locals_k=[
            LocalK(K_BEND_90, D_m=D_TRUNK),
            LocalK(k_t1_straight_to_t2, D_m=D_TRUNK),
        ],
    )

    # Branch a: straight + T1 branch loss
    r_a_duct = _r_duct(
        q_design=q_a,
        straight_runs=[Straight(L_m=4.0, D_m=D_BRANCH)],
        locals_k=[LocalK(k_t1_branch_a, D_m=D_BRANCH)],
    )

    # Branch c: reducer + straight + bend + T2 branch + hood
    r_c_duct = _r_duct(
        q_design=q_c,
        straight_runs=[Straight(L_m=3.2, D_m=D_BRANCH)],
        locals_k=[
            LocalK(K_REDUCER, D_m=D_BRANCH),
            LocalK(K_BEND_90, D_m=D_BRANCH),
            LocalK(k_t2_branch_c, D_m=D_BRANCH),
        ],
    )

    # Branch d: straight + bend + reducer + straight + bends + T2 branch + hood
    r_d_duct = _r_duct(
        q_design=q_d,
        straight_runs=[Straight(L_m=2.0, D_m=D_BRANCH), Straight(L_m=4.5, D_m=D_BRANCH)],
        locals_k=[
            LocalK(K_BEND_90, D_m=D_BRANCH),
            LocalK(K_REDUCER, D_m=D_BRANCH),
            LocalK(K_BEND_90, D_m=D_BRANCH),
            LocalK(K_BEND_90, D_m=D_BRANCH),
            LocalK(k_t2_branch_d, D_m=D_BRANCH),
        ],
    )

    r_a_dev = r_from_const_dp_at_design(dp_pa=DEVICE_DP_DESIGN[EDGE_A], Q_m3s_design=q_a) if DEVICE_DP_DESIGN[EDGE_A] > 0 else 0.0
    r_c_dev = r_from_const_dp_at_design(dp_pa=DEVICE_DP_DESIGN[EDGE_C], Q_m3s_design=q_c)
    r_d_dev = r_from_const_dp_at_design(dp_pa=DEVICE_DP_DESIGN[EDGE_D], Q_m3s_design=q_d)

    return {
        EDGE_TRUNK_T1: float(r_trunk_t1),
        EDGE_TRUNK_T2: float(r_trunk_t2),
        EDGE_A: float(r_a_duct + r_a_dev),
        EDGE_C: float(r_c_duct + r_c_dev),
        EDGE_D: float(r_d_duct + r_d_dev),
    }


def _compute_damper_k(
    q_design_by_edge: Dict[int, float],
    *,
    damper_model: str = REALCASE_DAMPER_MODEL_DEFAULT,
    damper_gamma: float = REALCASE_DAMPER_GAMMA_DEFAULT,
) -> Dict[int, float]:
    """
    Convert target damper dp at design point into model damper_k.
    solver uses: dp_damper = (damper_k / u(theta)^2) * Q^2
    -> damper_k = dp_target * u(theta_ref)^2 / Q_design^2
    """
    th = float(DAMPER_TARGET_THETA_DEG)
    if not (0.0 < th <= 90.0):
        raise ValueError("DAMPER_TARGET_THETA_DEG must be in (0, 90]")

    u = float(angle_to_u(th, model=str(damper_model), gamma=float(damper_gamma)))
    u = max(u, 1e-6)

    out: Dict[int, float] = {}
    for eid in CAV_EDGES:
        q = float(q_design_by_edge[eid])
        dp = float(DAMPER_TARGET_DP_PA[eid])
        if q <= 0.0:
            raise ValueError(f"q_design_by_edge[{eid}] must be > 0")
        if dp <= 0.0:
            raise ValueError(f"DAMPER_TARGET_DP_PA[{eid}] must be > 0")
        out[eid] = float(dp * (u * u) / (q * q))
    return out


def make_net(
    *,
    damper_model: str = REALCASE_DAMPER_MODEL_DEFAULT,
    damper_gamma: float = REALCASE_DAMPER_GAMMA_DEFAULT,
    q_design_by_edge: Dict[int, float] | None = None,
    fan_poly: tuple[float, float, float, float] | None = None,
) -> Network:
    q_design = dict(q_design_by_edge) if q_design_by_edge is not None else build_q_design()
    fan_poly_use = tuple(float(x) for x in (fan_poly if fan_poly is not None else REALCASE_FAN_POLY_DEFAULT))
    if len(fan_poly_use) != 4:
        raise ValueError('fan_poly must have 4 coefficients (a,b,c,d)')

    nodes = [
        Node(NODE_AMB, "amb_in"),
        Node(NODE_FAN_OUT, "fan_out"),
        Node(NODE_T1, "T1"),
        Node(NODE_T2, "T2"),
        Node(NODE_ROOM_A, "room_A"),
        Node(NODE_ROOM_C, "room_CD_c"),
        Node(NODE_ROOM_D, "room_CD_d"),
    ]

    r_by = _compute_edge_r(q_design)
    damper_k_by = _compute_damper_k(q_design, damper_model=damper_model, damper_gamma=damper_gamma)

    edges = [
        Edge(
            id=EDGE_FAN,
            frm=NODE_AMB,
            to=NODE_FAN_OUT,
            r=2.0,
            fan_poly=fan_poly_use,
            speed_ratio=1.0,
        ),
        Edge(id=EDGE_TRUNK_T1, frm=NODE_FAN_OUT, to=NODE_T1, r=r_by[EDGE_TRUNK_T1]),
        Edge(id=EDGE_TRUNK_T2, frm=NODE_T1, to=NODE_T2, r=r_by[EDGE_TRUNK_T2]),
        Edge(id=EDGE_A, frm=NODE_T1, to=NODE_ROOM_A, r=r_by[EDGE_A], damper_k=damper_k_by[EDGE_A], damper_u=1.0),
        Edge(id=EDGE_C, frm=NODE_T2, to=NODE_ROOM_C, r=r_by[EDGE_C], damper_k=damper_k_by[EDGE_C], damper_u=1.0),
        Edge(id=EDGE_D, frm=NODE_T2, to=NODE_ROOM_D, r=r_by[EDGE_D], damper_k=damper_k_by[EDGE_D], damper_u=1.0),
    ]

    return Network(nodes=nodes, edges=edges, ref_node=NODE_AMB)
