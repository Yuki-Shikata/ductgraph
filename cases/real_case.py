# cases/real_case.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

from ductgraph.model import Network, Node, Edge

# ============================================================
# IDs (keep stable: CLI / tests / post-processing rely on them)
# ============================================================
EDGE_FAN = 0
EDGE_TRUNK_T1 = 1
EDGE_TRUNK_T2 = 2
EDGE_A = 11
EDGE_C = 13
EDGE_D = 14

FAN_EDGE_IDS = [EDGE_FAN]

# Human-readable labels for reporting / logs
EDGE_LABELS: Dict[int, str] = {
    EDGE_FAN: "fan",
    EDGE_TRUNK_T1: "trunk_T1",
    EDGE_TRUNK_T2: "trunk_T2",
    EDGE_A: "a",
    EDGE_C: "c",
    EDGE_D: "d",
}

# ============================================================
# Setpoints (CMH) -> Design flows (m3/s)
# ============================================================
SETPOINTS_CMH: Dict[str, float] = {
    "a": 690.0,
    "c": 990.0,
    "d": 990.0,
}


def cmh_to_m3s(q_cmh: float) -> float:
    return float(q_cmh) / 3600.0


def build_q_design() -> Dict[int, float]:
    """edge_id -> q_design [m3/s] (positive)."""
    return {
        EDGE_A: cmh_to_m3s(SETPOINTS_CMH["a"]),
        EDGE_C: cmh_to_m3s(SETPOINTS_CMH["c"]),
        EDGE_D: cmh_to_m3s(SETPOINTS_CMH["d"]),
    }


# ---- metadata for CLI/commissioning ----
CAV_EDGES = [EDGE_A, EDGE_C, EDGE_D]
Q_DESIGN = build_q_design()


def choose_maxload_edge(q_design_by_edge: Dict[int, float], r_by_edge: Dict[int, float]) -> int:
    """
    Initial guess of maxload:
      pick the terminal edge with the largest design-point pressure loss proxy: dp ~ r * Q^2
    (This is only for initial selection; commissioning itself determines the final operating point.)
    """
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


# ============================================================
# Boundary pressures (fixed_p)
# ============================================================
# Two-room concept:
#   - a is in room_A
#   - c and d are in room_CD
#
# If in future room pressures differ (e.g., room is under negative pressure),
# set those node pressures here (Pa). The solver treats these nodes as fixed.
#
# IMPORTANT:
# - ref_node must be fixed (typically 0 Pa).
# - You may fix additional nodes (rooms) as needed.
ROOM_A_P = 0.0
ROOM_CD_P = 0.0

# Node IDs (keep stable)
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
    # c,d are different nodes but can share same fixed pressure => "same room" behavior
    NODE_ROOM_C: ROOM_CD_P,
    NODE_ROOM_D: ROOM_CD_P,
}


# ============================================================
# Network definition (tree, no loops)
# Fan -> trunk(350) -> T1 -> a(250) and b-trunk(350) -> T2 -> c(250), d(250)
# ============================================================
def make_net() -> Network:
    """
    Real-case network (tree, no loops).

    Nodes:
      0: amb (reference)
      1: fan_out
      2: tee1 (T1)
      3: tee2 (T2)
      101: room_A boundary
      103: room_CD boundary (c)
      104: room_CD boundary (d)

    Edges:
      0: fan (0->1) with fan_poly
      1: trunk_T1 (1->2)
      2: trunk_T2 (2->3)
      11: a branch (2->101) with CAV damper
      13: c branch (3->103) with hood+ CAV damper
      14: d branch (3->104) with hood+ CAV damper
    """

    # Air properties (currently used only for future extensions; r's are already aggregated)
    # rho = 1.2

    nodes = [
        Node(NODE_AMB, "amb_in"),
        Node(NODE_FAN_OUT, "fan_out"),
        Node(NODE_T1, "T1"),
        Node(NODE_T2, "T2"),
        Node(NODE_ROOM_A, "room_A"),
        Node(NODE_ROOM_C, "room_CD_c"),
        Node(NODE_ROOM_D, "room_CD_d"),
    ]

    # ------------------------------------------------------------
    # NOTE:
    # r values are the current aggregated-equivalent coefficients
    # that already made the realcase run converge in your latest run.
    # We will replace these with component-based r( Re, eps, ... )
    # later (Swamee–Jain, local losses library, reducers, etc.).
    # ------------------------------------------------------------
    r_fan_internal = 2.0
    r_trunk_t1 = 54.74317641335644
    r_trunk_t2 = 31.750030583087952
    r_a = 567.6644842205225
    r_c = 2676.2568985362186
    r_d = 2238.004682518058

    edges = [
        # Fan
        Edge(
            id=EDGE_FAN,
            frm=NODE_AMB,
            to=NODE_FAN_OUT,
            r=r_fan_internal,
            # fan_poly: ΔP = a Q^3 + b Q^2 + c Q + d  (solver expects its own unit convention)
            fan_poly=(0.0, -1.0, 0.0, 450.0),
            speed_ratio=1.0,
        ),

        # Trunks
        Edge(
            id=EDGE_TRUNK_T1,
            frm=NODE_FAN_OUT,
            to=NODE_T1,
            r=r_trunk_t1,
        ),
        Edge(
            id=EDGE_TRUNK_T2,
            frm=NODE_T1,
            to=NODE_T2,
            r=r_trunk_t2,
        ),

        # Terminals (CAV dampers are modeled via damper_k, damper_u)
        Edge(
            id=EDGE_A,
            frm=NODE_T1,
            to=NODE_ROOM_A,
            r=r_a,
            damper_k=72.0,
            damper_u=1.0,
        ),
        Edge(
            id=EDGE_C,
            frm=NODE_T2,
            to=NODE_ROOM_C,
            r=r_c,
            damper_k=11.0,
            damper_u=1.0,
        ),
        Edge(
            id=EDGE_D,
            frm=NODE_T2,
            to=NODE_ROOM_D,
            r=r_d,
            damper_k=15.0,
            damper_u=1.0,
        ),
    ]

    return Network(nodes=nodes, edges=edges, ref_node=NODE_AMB)


# ---- derived metadata (computed at import time) ----
_tmp = make_net()
_r_by_eid = {e.id: float(e.r) for e in _tmp.edges if e.id in Q_DESIGN}
MAXLOAD_EDGE = choose_maxload_edge(Q_DESIGN, _r_by_eid)
del _tmp, _r_by_eid


# Optional: scaling patterns (edge_id lists) for commissioning_scale
# (If you don't want partial operation now, you can ignore these.)
SCALING_CASES: List[Tuple[str, List[int]]] = [
    ("all", [EDGE_A, EDGE_C, EDGE_D]),
    ("a", [EDGE_A]),
    ("c", [EDGE_C]),
    ("d", [EDGE_D]),
    ("ac", [EDGE_A, EDGE_C]),
    ("ad", [EDGE_A, EDGE_D]),
    ("cd", [EDGE_C, EDGE_D]),
]
