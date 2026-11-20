from __future__ import annotations

import math
import os
import pathlib
import sys

# Ensure task directory is on sys.path for local helper modules like mechkit
_TASK_DIR = str(pathlib.Path(__file__).resolve().parent)
if _TASK_DIR not in sys.path:
    sys.path.insert(0, _TASK_DIR)
from typing import Any

import numpy as np

import mechkit as mk
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint

TAG = "clamp_grounding_clamp_fuse_carrier"

# Physical Materials and Colors
ANSI_GRAY = (0.50, 0.52, 0.55)        # NEMA 3R painted steel enclosure
COPPER_SHINE = (0.75, 0.45, 0.25)     # 110 copper busbar tab
BRONZE_CAST = (0.55, 0.42, 0.22)      # Cast bronze grounding clamp jaw
SAFETY_YELLOW = (0.92, 0.75, 0.12)    # Dielectric insulated grip handle
CERAMIC_WHITE = (0.90, 0.90, 0.88)    # High-dielectric glazed ceramic carrier body
CONTACT_BRASS = (0.80, 0.65, 0.25)    # Terminal clips and fuse ferrule contacts
SAFETY_ORANGE = (0.88, 0.35, 0.12)    # Anti-static parts tray

SCENE = "station"
TABLE_TOP = {"station": 0.748}
TOP = TABLE_TOP[SCENE]

# Station Positions
BOX_X, BOX_Y = 0.220, 0.000
BOX_BASE_Z = TOP + 0.055

# Ground busbar tab within disconnect enclosure (accessible on -Y side)
BUSBAR_POS = np.array([BOX_X, -0.060, TOP + 0.045])

# Fuse carrier initial seated position (centered inside enclosure)
FUSE_SEATED_POS = np.array([BOX_X, 0.000, TOP + 0.055])

# Grounding clamp initial staging position (on -Y prep bench)
CLAMP_INIT_POS = np.array([0.180, -0.180, TOP + 0.035])

# Safety parts tray position (on +Y bench)
TRAY_POS = np.array([0.220, 0.180, TOP + 0.015])


# -- Kit Setup -----------------------------------------------------------------

kit = mk.Kit()


# -- 1. Commercial Fused Disconnect Enclosure (NEMA 3R 60A Box) -----------------
kit.part(
    name="disconnect_box",
    pos=(BOX_X, BOX_Y, BOX_BASE_Z),
    boxes=[
        # Heavy-gauge steel backplate mounting chassis
        mk.box((0.140, 0.180, 0.010), at=(0.000, 0.000, -0.045), color=ANSI_GRAY),
        # Enclosure side walls (left -Y, right +Y, top +X, bottom -X)
        mk.box((0.140, 0.008, 0.080), at=(0.000, 0.086, 0.000), color=ANSI_GRAY),
        mk.box((0.140, 0.008, 0.080), at=(0.000, -0.086, 0.000), color=ANSI_GRAY),
        mk.box((0.008, 0.180, 0.080), at=(0.066, 0.000, 0.000), color=ANSI_GRAY),
        mk.box((0.008, 0.180, 0.080), at=(-0.066, 0.000, 0.000), color=ANSI_GRAY),
        # Open enclosure door (hinged on +X rim, swung open to 110 deg out of way)
        mk.box((0.140, 0.006, 0.080), at=(0.050, 0.145, 0.020), color=ANSI_GRAY),
        # Copper Ground Busbar Tab (6 mm thick x 25 mm wide x 40 mm high)
        mk.box((0.025, 0.006, 0.040), at=(0.000, -0.060, -0.010), color=COPPER_SHINE),
        # Upper and lower beryllium copper receiver spring clip terminals
        mk.box((0.020, 0.012, 0.018), at=(0.025, 0.000, -0.025), color=CONTACT_BRASS),
        mk.box((0.020, 0.012, 0.018), at=(-0.025, 0.000, -0.025), color=CONTACT_BRASS),
    ],
    mass=1.200,
)


# -- 2. Heavy Bronze Spring-Loaded Grounding Clamp -----------------------------
kit.part(
    name="grounding_clamp",
    pos=(CLAMP_INIT_POS[0], CLAMP_INIT_POS[1], CLAMP_INIT_POS[2]),
    boxes=[
        # Cast bronze duckbill upper clamp jaw (serrated contact pad)
        mk.box((0.035, 0.014, 0.008), at=(0.018, 0.000, 0.008), color=BRONZE_CAST),
        # Cast bronze duckbill lower clamp jaw
        mk.box((0.035, 0.014, 0.008), at=(0.018, 0.000, -0.008), color=BRONZE_CAST),
        # Central spring pivot barrel housing
        mk.box((0.020, 0.018, 0.024), at=(0.000, 0.000, 0.000), color=BRONZE_CAST),
        # Rear dielectric insulated grip handle post (26 mm wide x 18 mm thick x 55 mm high)
        mk.box((0.026, 0.018, 0.055), at=(-0.025, 0.000, 0.025), color=SAFETY_YELLOW),
    ],
    mass=0.350,
    contact_offset=0.0005,
    solver_position_iterations=128,
)


# -- 3. Ceramic 60A Pull-Out Fuse Carrier Assembly -----------------------------
kit.part(
    name="fuse_carrier",
    pos=(FUSE_SEATED_POS[0], FUSE_SEATED_POS[1], FUSE_SEATED_POS[2]),
    boxes=[
        # Glazed white ceramic main insulator block (75 mm long x 40 mm wide x 22 mm deep)
        mk.box((0.075, 0.040, 0.022), at=(0.000, 0.000, 0.000), color=CERAMIC_WHITE),
        # Dual terminal male bayonet contact blades (insert into enclosure spring clips)
        mk.box((0.014, 0.006, 0.020), at=(0.025, 0.000, -0.018), color=CONTACT_BRASS),
        mk.box((0.014, 0.006, 0.020), at=(-0.025, 0.000, -0.018), color=CONTACT_BRASS),
        # Front central ergonomic pull-handle loop (28 mm wide x 18 mm thick x 35 mm high)
        mk.box((0.028, 0.018, 0.035), at=(0.000, 0.000, 0.025), color=SAFETY_YELLOW),
    ],
    mass=0.160,
    contact_offset=0.0005,
    solver_position_iterations=128,
)


# -- 4. Molded Anti-Static Parts Safety Tray -----------------------------------
kit.part(
    name="safety_tray",
    pos=(TRAY_POS[0], TRAY_POS[1], TRAY_POS[2]),
    boxes=[
        # Tray base plate
        mk.box((0.120, 0.090, 0.008), at=(0.000, 0.000, -0.010), color=SAFETY_ORANGE),
        # Perimeter containment rim walls
        mk.box((0.120, 0.006, 0.025), at=(0.000, 0.042, 0.005), color=SAFETY_ORANGE),
        mk.box((0.120, 0.006, 0.025), at=(0.000, -0.042, 0.005), color=SAFETY_ORANGE),
        mk.box((0.006, 0.090, 0.025), at=(0.057, 0.000, 0.005), color=SAFETY_ORANGE),
        mk.box((0.006, 0.090, 0.025), at=(-0.057, 0.000, 0.005), color=SAFETY_ORANGE),
    ],
    mass=0.250,
)

OBJECTS = kit.objects()


# -- Scorer Implementation -----------------------------------------------------

def _on_step(objects: dict[str, Any]) -> None:
    M = mk.MEMORY
    step_idx = mk._EP.get("step", 0)

    gp, gq = kit.pose(objects, "grounding_clamp")
    fp, fq = kit.pose(objects, "fuse_carrier")

    # 1. Grounding clamp status
    # Jaws clamped over copper busbar tab
    dr_clamp = math.sqrt((gp[0] - (BUSBAR_POS[0] - 0.018)) ** 2 + (gp[1] - BUSBAR_POS[1]) ** 2)
    dz_clamp = abs(gp[2] - BUSBAR_POS[2])
    clamped = dr_clamp <= 0.015 and dz_clamp <= 0.015
    if clamped:
        M["clamp_grounded"] = True

    # Clamp lifted from table
    clamp_lifted = gp[2] > CLAMP_INIT_POS[2] + 0.020
    if clamp_lifted:
        M["clamp_lifted"] = True

    # 2. Hazard Fault Trigger Check (Rule 1: Hazard cleared before action)
    # If fuse carrier is displaced before clamp is firmly grounded, trip fault interlock
    dist_fuse_init = math.sqrt(
        (fp[0] - FUSE_SEATED_POS[0]) ** 2 +
        (fp[1] - FUSE_SEATED_POS[1]) ** 2 +
        (fp[2] - FUSE_SEATED_POS[2]) ** 2
    )
    if dist_fuse_init > 0.015 and not M.get("clamp_grounded", False):
        # UNGROUNDED EXTRACTION HAZARD TRIPPED!
        M["ungrounded_hazard_tripped"] = True

    # 3. Fuse Carrier Status
    # Extracted from enclosure spring clips (moved > 35 mm along +Z or +Y)
    fuse_extracted = dist_fuse_init > 0.035 and not M.get("ungrounded_hazard_tripped", False)
    if fuse_extracted:
        M["fuse_extracted"] = True

    # Placed in safety parts tray
    dr_tray = math.sqrt((fp[0] - TRAY_POS[0]) ** 2 + (fp[1] - TRAY_POS[1]) ** 2)
    dz_tray = abs(fp[2] - (TRAY_POS[2] + 0.018))
    fuse_in_tray = dr_tray <= 0.030 and dz_tray <= 0.020 and not M.get("ungrounded_hazard_tripped", False)
    if fuse_in_tray:
        M["fuse_in_tray"] = True

    # 4. Hands cleared
    hands_clear = bool(
        _released(objects["grounding_clamp"], 0.040)
        and _released(objects["fuse_carrier"], 0.040)
    )

    # 5. Success hold counter
    if (
        clamped
        and fuse_in_tray
        and hands_clear
        and not failed(objects)
        and not M.get("ungrounded_hazard_tripped", False)
    ):
        hold = M.get("hold_timer", 0) + 1
        M["hold_timer"] = hold
    else:
        M["hold_timer"] = 0

    if step_idx % 40 == 0:
        print(
            f"[{TAG}] step={step_idx} dr_c={dr_clamp:.3f} dz_c={dz_clamp:.3f} "
            f"dist_f={dist_fuse_init:.3f} dr_tray={dr_tray:.3f} dz_tray={dz_tray:.3f} "
            f"grounded={clamped} hazard={M.get('ungrounded_hazard_tripped', False)} "
            f"in_tray={fuse_in_tray} clear={hands_clear} hold={M.get('hold_timer', 0)} "
            f"score={score(objects)}",
            flush=True,
        )


kit.on_step(_on_step)


# -- Scorer Helpers ------------------------------------------------------------

def _released(obj_state: Any, margin: float = 0.040) -> bool:
    """True if no robot gripper or finger is within margin of the object."""
    tcps = mk.tcps()
    if not tcps:
        return True
    pos = np.asarray(obj_state.pos, dtype=float)
    return all(np.linalg.norm(np.asarray(tcp, dtype=float) - pos) >= margin for tcp in tcps)


def clamp_grounded(objects: dict) -> bool:
    """True if grounding clamp jaw is currently securely clamped over copper busbar tab."""
    gp, _ = kit.pose(objects, "grounding_clamp")
    dr = math.sqrt((gp[0] - (BUSBAR_POS[0] - 0.018)) ** 2 + (gp[1] - BUSBAR_POS[1]) ** 2)
    dz = abs(gp[2] - BUSBAR_POS[2])
    return bool(dr <= 0.015 and dz <= 0.015)


def fuse_in_tray(objects: dict) -> bool:
    """True if fuse carrier is currently safely nested in parts tray."""
    if not clamp_grounded(objects):
        return False
    if mk.MEMORY.get("ungrounded_hazard_tripped", False):
        return False
    fp, _ = kit.pose(objects, "fuse_carrier")
    dr = math.sqrt((fp[0] - TRAY_POS[0]) ** 2 + (fp[1] - TRAY_POS[1]) ** 2)
    dz = abs(fp[2] - (TRAY_POS[2] + 0.018))
    return bool(dr <= 0.030 and dz <= 0.020)


def failed(objects: dict) -> bool:
    """True if enclosure tipped, components dropped off bench, or hazard tripped."""
    if mk.MEMORY.get("ungrounded_hazard_tripped", False):
        return True

    box_p, box_q = kit.pose(objects, "disconnect_box")
    if abs(box_p[0] - BOX_X) > 0.030 or abs(box_p[1] - BOX_Y) > 0.030:
        return True
    b_mat = mk.quat_to_mat(box_q)
    if b_mat[2, 2] < 0.95:  # Box tipped > 18 deg
        return True

    for name in ("grounding_clamp", "fuse_carrier"):
        p, _ = kit.pose(objects, name)
        if p[2] < TOP - 0.030:  # Dropped below table edge
            return True
        if math.sqrt(p[0] ** 2 + p[1] ** 2) > 0.65:
            return True
    return False


def score(objects: dict) -> float:
    """Staged scoring with partial credit: 0.25 -> 0.50 -> 0.75 -> 1.00."""
    if failed(objects):
        return 0.0

    M = mk.MEMORY
    if not clamp_grounded(objects):
        return 0.25 if M.get("clamp_lifted", False) else 0.0

    # Grounding clamp is currently secured to copper busbar tab
    dist_fuse_init = math.sqrt(
        (objects["fuse_carrier"].pos[0] - FUSE_SEATED_POS[0]) ** 2 +
        (objects["fuse_carrier"].pos[1] - FUSE_SEATED_POS[1]) ** 2 +
        (objects["fuse_carrier"].pos[2] - FUSE_SEATED_POS[2]) ** 2
    )
    if dist_fuse_init <= 0.025:
        return 0.50  # Grounded, fuse not yet extracted

    if not fuse_in_tray(objects):
        return 0.75  # Grounded and fuse extracted from clips

    hands_clear = (
        _released(objects["grounding_clamp"], 0.040)
        and _released(objects["fuse_carrier"], 0.040)
    )
    hold = M.get("hold_timer", 0)
    if hands_clear and hold >= 20:
        return 1.00

    return 0.75


def success(objects: dict) -> bool:
    return score(objects) >= 1.0


# Exported constants
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0


# -- Scripted Demo Waypoints ---------------------------------------------------

def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("DISCONNECT_CHEAT", "")

    if cheat == "nothing":
        left = [Waypoint(10.0, (0.180, 0.200, TOP + 0.150), FINGER_OPEN, "park_left")]
        right = [Waypoint(10.0, (0.180, -0.200, TOP + 0.150), FINGER_OPEN, "park_right")]
        return {"left": left, "right": right}

    # Clamp handle grip coordinates
    cg_x = CLAMP_INIT_POS[0] - 0.025
    cg_y = CLAMP_INIT_POS[1]
    cg_z = CLAMP_INIT_POS[2] + 0.025
    CARRY_Z = TOP + 0.140

    # Busbar tab attachment coordinates for clamp handle
    # When jaws are over busbar at BUSBAR_POS, handle is at BUSBAR_POS - [0.018, 0, 0]
    attach_x = BUSBAR_POS[0] - 0.018 - 0.025
    attach_y = BUSBAR_POS[1]
    attach_z = BUSBAR_POS[2] + 0.025

    # Fuse pull handle coordinates
    fh_x = FUSE_SEATED_POS[0]
    fh_y = FUSE_SEATED_POS[1]
    fh_z = FUSE_SEATED_POS[2] + 0.025

    # Tray placement coordinates
    tray_place_z = TRAY_POS[2] + 0.035

    # Left arm: grasps grounding clamp, carries and attaches jaws onto busbar, then retracts
    # Right arm: waits for grounding, grasps fuse carrier, withdraws it, places in tray
    if cheat == "no_ground":
        # Right arm pulls fuse WITHOUT grounding (trips hazard fault detector -> score 0.0)
        left = [Waypoint(10.0, (0.180, -0.200, CARRY_Z), FINGER_OPEN, "left_park")]
        right = [
            Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_OPEN, "fuse_approach"),
            Waypoint(0.8, (fh_x, fh_y, fh_z), FINGER_CLOSED, "fuse_grasp"),
            Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_extract_unprotected"),
            Waypoint(8.0, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_hold"),
        ]
        return {"left": left, "right": right}

    left = [
        # 1. Approach and grasp grounding clamp handle
        Waypoint(1.2, (cg_x, cg_y, CARRY_Z), FINGER_OPEN, "clamp_approach"),
        Waypoint(0.8, (cg_x, cg_y, cg_z), FINGER_CLOSED, "clamp_grasp"),
        Waypoint(1.0, (cg_x, cg_y, CARRY_Z), FINGER_CLOSED, "clamp_lift"),
        # Carry clamp toward enclosure busbar tab
        Waypoint(0.8, (attach_x - 0.040, attach_y, CARRY_Z), FINGER_CLOSED, "clamp_carry_1"),
        Waypoint(0.8, (attach_x - 0.040, attach_y, attach_z), FINGER_CLOSED, "clamp_align"),
        # Push duckbill jaws over copper busbar tab
        Waypoint(1.2, (attach_x, attach_y, attach_z), FINGER_CLOSED, "clamp_attach"),
        # Release handle so spring jaws bite into copper busbar
        Waypoint(0.8, (attach_x, attach_y, attach_z), FINGER_OPEN, "clamp_release"),
        Waypoint(1.0, (attach_x - 0.050, attach_y, CARRY_Z), FINGER_OPEN, "clamp_clear"),
        Waypoint(1.0, (0.180, -0.220, CARRY_Z), FINGER_OPEN, "left_retract"),
        Waypoint(12.0, (0.180, -0.220, CARRY_Z), FINGER_OPEN, "left_park"),
    ]

    if cheat == "ground_only":
        right = [Waypoint(16.0, (0.180, 0.200, CARRY_Z), FINGER_OPEN, "right_park")]
        return {"left": left, "right": right}

    # Right arm: waits for grounding clamp attachment (t = 8.0s), then extracts fuse
    right = [
        # Park until grounding clamp is attached to busbar
        Waypoint(8.0, (0.180, 0.200, CARRY_Z), FINGER_OPEN, "right_wait"),
        # Approach and grasp ceramic fuse carrier pull-handle
        Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_OPEN, "fuse_approach"),
        Waypoint(0.8, (fh_x, fh_y, fh_z), FINGER_CLOSED, "fuse_grasp"),
        # Withdraw fuse carrier straight up out of spring-clip terminals (20 N draw)
        Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_extract"),
    ]

    if cheat == "no_tray":
        right += [
            Waypoint(10.0, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_hold"),
        ]
        return {"left": left, "right": right}

    right += [
        # Carry fuse carrier over to safety parts tray
        Waypoint(1.0, (TRAY_POS[0], TRAY_POS[1], CARRY_Z), FINGER_CLOSED, "tray_approach"),
        Waypoint(1.0, (TRAY_POS[0], TRAY_POS[1], tray_place_z), FINGER_CLOSED, "tray_place"),
        # Release fuse carrier into tray and retract clear
        Waypoint(0.8, (TRAY_POS[0], TRAY_POS[1], tray_place_z), FINGER_OPEN, "fuse_release"),
        Waypoint(1.0, (TRAY_POS[0], TRAY_POS[1], CARRY_Z), FINGER_OPEN, "tray_clear"),
        Waypoint(1.0, (0.180, 0.220, CARRY_Z), FINGER_OPEN, "right_retract"),
        Waypoint(8.0, (0.180, 0.220, CARRY_Z), FINGER_OPEN, "right_park"),
    ]

    return {"left": left, "right": right}
