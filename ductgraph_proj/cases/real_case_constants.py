from __future__ import annotations

from typing import Dict, List, Tuple

# IDs
EDGE_FAN = 0
EDGE_TRUNK_T1 = 1
EDGE_TRUNK_T2 = 2
EDGE_A = 11
EDGE_C = 13
EDGE_D = 14

FAN_EDGE_IDS = [EDGE_FAN]
CAV_EDGES = [EDGE_A, EDGE_C, EDGE_D]

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
