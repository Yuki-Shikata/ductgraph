# cases/real_case.py
from __future__ import annotations

from typing import Dict, List, Tuple

from ductgraph.model import Network, Node, Edge
from ductgraph.ductloss import (
    Straight,
    LocalK,
    r_from_segments_design_point,
    r_from_const_dp_at_design,
    tee_K_from_ratio,
)

# IDs
EDGE_FAN = 0
EDGE_TRUNK_T1 = 1
EDGE_TRUNK_T2 = 2
EDGE_A = 11
EDGE_C = 13
EDGE_D = 14

FAN_EDGE_IDS = [EDGE_FAN]

EDGE_LABELS: Dict[int, str] = {
    EDGE_FAN: "fan",
    EDGE_TRUNK_T1: "trunk_T1",
    EDGE_TRUNK_T2: "trunk_T2",
    EDGE_A: "a",
    EDGE_C: "c",
    EDGE_D: "d",
}

SETPOINTS_CMH: Dict[str, float] = {
    "a": 690.0,
    "c": 990.0,
    "d": 990.0,
}

# Real-case tolerance defaults (equipment/operation side):
# - eps_under_rel: acceptance margin used by commissioning logic
# - tol_warn_rel: display threshold for WARN/UNDER split
EPS_UNDER_REL_DEFAULT = 0.01
TOL_WARN_REL_DEFAULT = 0.03


def cmh_to_m3s(q_cmh: float) -> float:
    return float(q_cmh) / 3600.0


def build_q_design() -> Dict[int, float]:
    return {
        EDGE_A: cmh_to_m3s(SETPOINTS_CMH["a"]),
        EDGE_C: cmh_to_m3s(SETPOINTS_CMH["c"]),
        EDGE_D: cmh_to_m3s(SETPOINTS_CMH["d"]),
    }


CAV_EDGES = [EDGE_A, EDGE_C, EDGE_D]
Q_DESIGN = build_q_design()


def choose_maxload_edge(q_design_by_edge: Dict[int, float], r_by_edge: Dict[int, float]) -> int:
    best_eid = None
    best_dp = None
    for eid, q in q_design_by_edge.items():
        r = float(r_by_edge.get(eid, 0.0))
        dp = r * (q ** 2)
        if best_dp is None or dp > best_dp:
            best_dp = dp
            best_eid = eid
    assert best_eid is not None
    return int(best_eid)


ROOM_A_P = 0.0
ROOM_CD_P = 0.0

NODE_AMB = 0
NODE_FAN_OUT = 1
NODE_T1 = 2
NODE_T2 = 3
NODE_ROOM_A = 101
NODE_ROOM_C = 103
NODE_ROOM_D = 104

FIXED_P: Dict[int, float] = {
    NODE_AMB: 0.0,
    NODE_ROOM_A: ROOM_A_P,
    NODE_ROOM_C: ROOM_CD_P,
    NODE_ROOM_D: ROOM_CD_P,
}

# --- physical constants / assumptions for design-point equivalent resistance ---
AIR_RHO = 1.2
AIR_NU = 1.5e-5
DUCT_ROUGHNESS_M = 0.00015

D_TRUNK = 0.35
D_BRANCH = 0.25

K_BEND_90 = 0.25
K_REDUCER = 0.20

# Damper practical tuning target:
# at design flow, each terminal should consume a meaningful static pressure
# at a realistic commissioning angle band center.
DAMPER_TARGET_THETA_DEG = 75.0
DAMPER_TARGET_DP_PA = {
    EDGE_A: 300.0,
    EDGE_C: 140.0,
    EDGE_D: 140.0,
}

# terminal device dp at design [Pa]
# c,d hood dp=69Pa from practical sheet; a has no hood here
DEVICE_DP_DESIGN: Dict[int, float] = {
    EDGE_A: 0.0,
    EDGE_C: 69.0,
    EDGE_D: 69.0,
}

# Tee K tables (design-flow-ratio interpolation)
# These are fixed design-point surrogates for current solver (no per-iteration K update).
TEE_K_BRANCH_TABLE = [
    # practical diverging-tee branch K (order): ~0.55-1.0 around design range
    (0.0, 0.95), (0.1, 0.90), (0.2, 0.85), (0.3, 0.80), (0.4, 0.75),
    (0.5, 0.70), (0.6, 0.67), (0.7, 0.64), (0.8, 0.61), (0.9, 0.58), (1.0, 0.55),
]
TEE_K_STRAIGHT_TABLE = [
    # practical tee-through K at comparable velocity level: ~0.06-0.30
    (0.0, 0.06), (0.1, 0.08), (0.2, 0.10), (0.3, 0.12), (0.4, 0.14),
    (0.5, 0.16), (0.6, 0.18), (0.7, 0.21), (0.8, 0.24), (0.9, 0.27), (1.0, 0.30),
]


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


def _compute_edge_r() -> Dict[int, float]:
    q_a = float(Q_DESIGN[EDGE_A])
    q_c = float(Q_DESIGN[EDGE_C])
    q_d = float(Q_DESIGN[EDGE_D])
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


def _compute_damper_k() -> Dict[int, float]:
    """
    Convert target damper dp at design point into model damper_k.
    solver uses: dp_damper = (damper_k / u(theta)^2) * Q^2
    -> damper_k = dp_target * u(theta_ref)^2 / Q_design^2
    """
    import math

    th = float(DAMPER_TARGET_THETA_DEG)
    if not (0.0 < th <= 90.0):
        raise ValueError("DAMPER_TARGET_THETA_DEG must be in (0, 90]")
    u = math.sin(math.radians(th))
    u = max(u, 1e-6)

    out: Dict[int, float] = {}
    for eid in (EDGE_A, EDGE_C, EDGE_D):
        q = float(Q_DESIGN[eid])
        dp = float(DAMPER_TARGET_DP_PA[eid])
        if q <= 0.0:
            raise ValueError(f"Q_DESIGN[{eid}] must be > 0")
        if dp <= 0.0:
            raise ValueError(f"DAMPER_TARGET_DP_PA[{eid}] must be > 0")
        out[eid] = float(dp * (u * u) / (q * q))
    return out


def make_net() -> Network:
    nodes = [
        Node(NODE_AMB, "amb_in"),
        Node(NODE_FAN_OUT, "fan_out"),
        Node(NODE_T1, "T1"),
        Node(NODE_T2, "T2"),
        Node(NODE_ROOM_A, "room_A"),
        Node(NODE_ROOM_C, "room_CD_c"),
        Node(NODE_ROOM_D, "room_CD_d"),
    ]

    r_by = _compute_edge_r()
    damper_k_by = _compute_damper_k()

    edges = [
        Edge(
            id=EDGE_FAN,
            frm=NODE_AMB,
            to=NODE_FAN_OUT,
            r=2.0,
            fan_poly=(-453.49632, 522.288, -257.04, 978.8),
            speed_ratio=1.0,
        ),
        Edge(id=EDGE_TRUNK_T1, frm=NODE_FAN_OUT, to=NODE_T1, r=r_by[EDGE_TRUNK_T1]),
        Edge(id=EDGE_TRUNK_T2, frm=NODE_T1, to=NODE_T2, r=r_by[EDGE_TRUNK_T2]),
        Edge(id=EDGE_A, frm=NODE_T1, to=NODE_ROOM_A, r=r_by[EDGE_A], damper_k=damper_k_by[EDGE_A], damper_u=1.0),
        Edge(id=EDGE_C, frm=NODE_T2, to=NODE_ROOM_C, r=r_by[EDGE_C], damper_k=damper_k_by[EDGE_C], damper_u=1.0),
        Edge(id=EDGE_D, frm=NODE_T2, to=NODE_ROOM_D, r=r_by[EDGE_D], damper_k=damper_k_by[EDGE_D], damper_u=1.0),
    ]

    return Network(nodes=nodes, edges=edges, ref_node=NODE_AMB)


_tmp = make_net()
_r_by_eid = {e.id: float(e.r) for e in _tmp.edges if e.id in Q_DESIGN}
MAXLOAD_EDGE = choose_maxload_edge(Q_DESIGN, _r_by_eid)
del _tmp, _r_by_eid

SCALING_CASES: List[Tuple[str, List[int]]] = [
    ("all", [EDGE_A, EDGE_C, EDGE_D]),
    ("a", [EDGE_A]),
    ("c", [EDGE_C]),
    ("d", [EDGE_D]),
    ("ac", [EDGE_A, EDGE_C]),
    ("ad", [EDGE_A, EDGE_D]),
    ("cd", [EDGE_C, EDGE_D]),
]

