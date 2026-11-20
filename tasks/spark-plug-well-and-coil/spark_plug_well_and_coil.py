"""RoboCurve Benchmark Task: spark-plug-well-and-coil
Automotive Service Technicians and Mechanics (SOC 49-3023, 760,000+ US workers).

Task:
Install a replacement spark plug into a recessed cylinder head well using a retention
socket extension tool, thread it home into the combustion chamber seat, withdraw the
tool to cradle, and install a coil-on-plug (COP) ignition unit with its rubber boot
sealing over the ceramic terminal and mounting flange resting flush on the boss.
"""

from __future__ import annotations

import math
import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import GREY, Box, ObjectState

import mechkit as mk  # noqa: E402

# Environment colors
ALU = (0.75, 0.77, 0.80)
STEEL = (0.60, 0.62, 0.65)
WHITE = (0.95, 0.95, 0.95)
BLACK = (0.12, 0.12, 0.12)
BRASS = (0.80, 0.65, 0.25)
BOOT_BLACK = (0.15, 0.15, 0.16)

# Table top height
TOP = 0.7475

# Geometry definitions
CYL_HEAD_XY = (0.260, 0.000)
CYL_HEAD_SIZE = (0.180, 0.140, 0.040)
WELL_XY = (0.260, 0.000)
WELL_DEPTH = 0.030
WELL_BORE = 0.026
WELL_TOP_Z = TOP + CYL_HEAD_SIZE[2]       # 0.7875 m
WELL_SEAT_Z = WELL_TOP_Z - WELL_DEPTH     # 0.7575 m

BOSS_XY = (0.295, 0.000)
BOSS_Z = WELL_TOP_Z
BOSS_SIZE = (0.024, 0.024, 0.008)

PLUG_TRAY_XY = (0.220, -0.120)
PLUG_TRAY_Z = TOP + 0.015

TOOL_CRADLE_XY = (0.200, -0.200)
TOOL_CRADLE_Z = TOP + 0.015

COIL_STAND_XY = (0.220, 0.180)
COIL_STAND_Z = TOP + 0.015

_T22 = math.tan(math.radians(22.5))


def _oct_rod(across, z0, h, color=None):
    return [mk.box((across, across * _T22, h), at=(0, 0, z0 + h / 2), color=color, yaw=k * math.pi / 4)
            for k in range(4)]


def _square_ring(outer, inner, h, z0, color=None):
    w = (outer - inner) / 2
    c = (outer + inner) / 4
    return [
        mk.box((outer, w, h), at=(0, c, z0 + h / 2), color=color),
        mk.box((outer, w, h), at=(0, -c, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(c, 0, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(-c, 0, z0 + h / 2), color=color),
    ]


kit = mk.Kit()

# 1. Fixed Cylinder Head Section & Recessed Well
kit.part("cyl_head", (CYL_HEAD_XY[0], CYL_HEAD_XY[1], TOP),
         [mk.box((CYL_HEAD_SIZE[0], (CYL_HEAD_SIZE[1] - WELL_BORE) / 2, CYL_HEAD_SIZE[2]),
                 at=(0, (CYL_HEAD_SIZE[1] + WELL_BORE) / 4, CYL_HEAD_SIZE[2] / 2), color=ALU),
          mk.box((CYL_HEAD_SIZE[0], (CYL_HEAD_SIZE[1] - WELL_BORE) / 2, CYL_HEAD_SIZE[2]),
                 at=(0, -(CYL_HEAD_SIZE[1] + WELL_BORE) / 4, CYL_HEAD_SIZE[2] / 2), color=ALU),
          mk.box(((CYL_HEAD_SIZE[0] - WELL_BORE) / 2, WELL_BORE, CYL_HEAD_SIZE[2]),
                 at=((CYL_HEAD_SIZE[0] + WELL_BORE) / 4, 0, CYL_HEAD_SIZE[2] / 2), color=ALU),
          mk.box(((CYL_HEAD_SIZE[0] - WELL_BORE) / 2, WELL_BORE, CYL_HEAD_SIZE[2]),
                 at=(-(CYL_HEAD_SIZE[0] + WELL_BORE) / 4, 0, CYL_HEAD_SIZE[2] / 2), color=ALU),
          mk.box((WELL_BORE, WELL_BORE, WELL_SEAT_Z - TOP),
                 at=(0, 0, (WELL_SEAT_Z - TOP) / 2), color=ALU),
          mk.box(BOSS_SIZE, at=(BOSS_XY[0] - CYL_HEAD_XY[0], 0, CYL_HEAD_SIZE[2] + BOSS_SIZE[2] / 2), color=STEEL)],
         mass=None, color=ALU)

# 2. Spark Plug
PLUG_STEM_D = 0.014
PLUG_STEM_H = 0.012
PLUG_HEX_W = 0.016
PLUG_HEX_H = 0.006
PLUG_INS_D = 0.0105
PLUG_INS_H = 0.018
PLUG_TERM_D = 0.004
PLUG_TERM_H = 0.006
PLUG_TOTAL_H = PLUG_STEM_H + PLUG_HEX_H + PLUG_INS_H + PLUG_TERM_H  # 0.042 m

PLUG_START_POS = (PLUG_TRAY_XY[0], PLUG_TRAY_XY[1], PLUG_TRAY_Z)
kit.part("spark_plug", PLUG_START_POS,
         [*_oct_rod(PLUG_STEM_D, 0.0, PLUG_STEM_H, color=BLACK),
          *_oct_rod(PLUG_HEX_W, PLUG_STEM_H, PLUG_HEX_H, color=STEEL),
          *_oct_rod(PLUG_INS_D, PLUG_STEM_H + PLUG_HEX_H, PLUG_INS_H, color=WHITE),
          *_oct_rod(PLUG_TERM_D, PLUG_STEM_H + PLUG_HEX_H + PLUG_INS_H, PLUG_TERM_H, color=BRASS)],
         mass=0.035, color=WHITE, contact_offset=0.0005)

# 3. Spark Plug Socket Tool (hex socket, extension tube, T-handle)
SOCKET_OD = 0.022
SOCKET_HEX_ID = 0.0165
SOCKET_CUP_H = 0.016
TOOL_TUBE_D = 0.012
TOOL_TUBE_H = 0.025
HANDLE_BAR = (0.014, 0.065, 0.012)

TOOL_START_POS = (TOOL_CRADLE_XY[0], TOOL_CRADLE_XY[1], TOOL_CRADLE_Z)
kit.part("socket_tool", TOOL_START_POS,
         [*_square_ring(SOCKET_OD, SOCKET_HEX_ID, SOCKET_CUP_H, 0.0, color=STEEL),
          mk.box((SOCKET_OD, SOCKET_OD, 0.004), at=(0, 0, SOCKET_CUP_H + 0.002), color=STEEL),
          *_oct_rod(TOOL_TUBE_D, SOCKET_CUP_H + 0.004, TOOL_TUBE_H, color=STEEL),
          mk.box(HANDLE_BAR, at=(0, 0, SOCKET_CUP_H + 0.004 + TOOL_TUBE_H + HANDLE_BAR[2] / 2), color=BLACK)],
         mass=0.075, color=STEEL, contact_offset=0.0005)

# 4. Ignition Coil Unit (silicone boot, head with connector, mounting ear)
BOOT_OD = 0.018
BOOT_H = 0.024
COIL_HEAD_SIZE = (0.045, 0.032, 0.020)
EAR_SIZE = (0.022, 0.018, 0.006)
EAR_OFFSET_X = 0.035

TARGET_COIL_BOTTOM_Z = WELL_TOP_Z - BOOT_H  # 0.7635 m

COIL_START_POS = (COIL_STAND_XY[0], COIL_STAND_XY[1], COIL_STAND_Z)
kit.part("ignition_coil", COIL_START_POS,
         [*_oct_rod(BOOT_OD, 0.0, BOOT_H, color=BOOT_BLACK),
          mk.box(COIL_HEAD_SIZE, at=(0, 0, BOOT_H + COIL_HEAD_SIZE[2] / 2), color=BLACK),
          mk.box(EAR_SIZE, at=(EAR_OFFSET_X, 0, BOOT_H + EAR_SIZE[2] / 2), color=STEEL)],
         mass=0.090, color=BLACK, contact_offset=0.0005)

# Catches:
# A. Tool holds spark plug when captured
_TOOL_GRIP_PLUG = mk._T(None, (0.0, 0.0, -0.012))
kit.catch("tool_plug_catch", "spark_plug", _TOOL_GRIP_PLUG, "socket_tool", mk._T())

# B. Well seat holds spark plug when seated home
_PLUG_SEAT_REF = mk._T(None, (WELL_XY[0], WELL_XY[1], WELL_SEAT_Z))
kit.catch("plug_seated_catch", "spark_plug", _PLUG_SEAT_REF)

# C. Coil unit seated over plug terminal and boss
_COIL_SEATED_REF = mk._T(None, (WELL_XY[0], WELL_XY[1], TARGET_COIL_BOTTOM_Z))
kit.catch("coil_seated_catch", "ignition_coil", _COIL_SEATED_REF)

# Decorative scenery
OBJECTS = [
    *kit.objects(),
    Box("plug_tray", pos=(PLUG_TRAY_XY[0], PLUG_TRAY_XY[1], TOP + 0.0075), size=(0.060, 0.060, 0.015), color=BLACK),
    Box("tool_cradle", pos=(TOOL_CRADLE_XY[0], TOOL_CRADLE_XY[1], TOP + 0.0075), size=(0.060, 0.060, 0.015), color=BLACK),
    Box("coil_stand", pos=(COIL_STAND_XY[0], COIL_STAND_XY[1], TOP + 0.0075), size=(0.060, 0.060, 0.015), color=BLACK),
]


# Step rules
def _simulation_rules(objects):
    plug_st = objects["spark_plug"]
    plug_pos = kit.point(objects, "spark_plug", (0, 0, 0))

    # Dropped defect check: if plug touches table deck outside tray and well
    d_tray = math.dist((plug_pos[0], plug_pos[1]), PLUG_TRAY_XY)
    d_well = math.dist((plug_pos[0], plug_pos[1]), WELL_XY)
    if plug_pos[2] < TOP + 0.006 and d_tray > 0.045 and d_well > 0.035:
        mk.MEMORY["plug_dropped_defect"] = True

    tool_st = objects["socket_tool"]
    tool_pos = kit.point(objects, "socket_tool", (0, 0, 0))

    # 1. Capture plug in tray: when tool socket cup is lowered over plug
    d_tool_plug_xy = math.dist((tool_pos[0], tool_pos[1]), (plug_pos[0], plug_pos[1]))
    if d_tool_plug_xy < 0.010 and 0.005 <= (tool_pos[2] - plug_pos[2]) <= 0.020:
        if not mk.MEMORY.get("plug_captured"):
            mk.MEMORY["plug_captured"] = True
            kit.engage("tool_plug_catch")

    # 2. Seat plug in well: when plug is at well bottom and on axis
    d_well_plug = math.dist((plug_pos[0], plug_pos[1]), WELL_XY)
    if d_well_plug < 0.005 and abs(plug_pos[2] - WELL_SEAT_Z) < 0.005:
        if not mk.MEMORY.get("plug_seated"):
            mk.MEMORY["plug_seated"] = True
            kit.engage("plug_seated_catch")

    # Tool withdrawn above well mouth: releases tool catch
    if mk.MEMORY.get("plug_seated") and tool_pos[2] > WELL_TOP_Z + 0.010:
        kit.disengage("tool_plug_catch")
        mk.MEMORY["tool_withdrawn"] = True

    # 3. Coil insertion and seating
    coil_st = objects["ignition_coil"]
    coil_pos = kit.point(objects, "ignition_coil", (0, 0, 0))
    d_coil_well_xy = math.dist((coil_pos[0], coil_pos[1]), WELL_XY)
    if mk.MEMORY.get("plug_seated") and mk.MEMORY.get("tool_withdrawn"):
        if d_coil_well_xy < 0.012 and coil_pos[2] < WELL_TOP_Z + 0.010:
            mk.MEMORY["coil_entered"] = True
        if d_coil_well_xy < 0.005 and abs(coil_pos[2] - TARGET_COIL_BOTTOM_Z) < 0.005:
            mk.MEMORY["coil_seated"] = True
            kit.engage("coil_seated_catch")


kit.on_step(_simulation_rules)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def score(objects) -> float:
    if mk.MEMORY.get("plug_dropped_defect"):
        return 0.0

    plug_st = objects["spark_plug"]
    tool_st = objects["socket_tool"]
    coil_st = objects["ignition_coil"]

    plug_pos = kit.point(objects, "spark_plug", (0, 0, 0))
    tool_pos = kit.point(objects, "socket_tool", (0, 0, 0))
    coil_pos = kit.point(objects, "ignition_coil", (0, 0, 0))

    # Check 1.0 condition: plug seated, tool clear, coil seated, arms clear
    d_well_plug = math.dist((plug_pos[0], plug_pos[1]), WELL_XY)
    plug_is_home = (d_well_plug <= 0.003 and abs(plug_pos[2] - WELL_SEAT_Z) <= 0.003)

    tool_is_clear = (tool_pos[2] >= WELL_TOP_Z + 0.010 or math.dist((tool_pos[0], tool_pos[1]), WELL_XY) >= 0.040)

    d_well_coil = math.dist((coil_pos[0], coil_pos[1]), WELL_XY)
    coil_is_home = (d_well_coil <= 0.003 and abs(coil_pos[2] - TARGET_COIL_BOTTOM_Z) <= 0.003)

    arms_clear = _released(plug_st) and _released(coil_st)

    if plug_is_home and tool_is_clear and coil_is_home and arms_clear:
        return 1.0

    if plug_is_home and (coil_is_home or mk.MEMORY.get("coil_entered")):
        return 0.75

    if plug_is_home:
        return 0.50 if tool_is_clear else 0.25

    if mk.MEMORY.get("plug_captured") or plug_pos[2] > PLUG_TRAY_Z + 0.010:
        return 0.25

    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    coil_pos = objects["ignition_coil"].pos
    return float(math.dist((coil_pos[0], coil_pos[1]), WELL_XY) + abs(coil_pos[2] - TARGET_COIL_BOTTOM_Z))


# Demo
TIP = 0.013
TRAVEL = 0.865
FINGER_OPEN = 0.045
FINGER_CLOSED = 0.008


def demo():
    # Right Arm: handles socket tool and spark plug
    (tx, ty, tz) = TOOL_START_POS
    th_z = tz + SOCKET_CUP_H + 0.004 + TOOL_TUBE_H + HANDLE_BAR[2] / 2
    grip_to_bottom = (th_z - tz) + TIP

    r_plan = [
        Waypoint(2.0, (tx, ty, TRAVEL), FINGER_OPEN, "over socket tool"),
        Waypoint(1.5, (tx, ty, th_z + TIP), FINGER_OPEN, "lower to tool handle"),
        Waypoint(0.8, (tx, ty, th_z + TIP), FINGER_CLOSED, "grip handle"),
        Waypoint(1.2, (tx, ty, TRAVEL), FINGER_CLOSED, "lift socket tool"),
    ]

    # 2. Carry over spark plug in tray, press down to capture
    (px, py, pz) = PLUG_START_POS
    r_plan += [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_CLOSED, "carry over spark plug"),
        Waypoint(1.5, (px, py, pz + 0.012 + grip_to_bottom), FINGER_CLOSED, "descend over plug"),
        Waypoint(0.8, (px, py, pz + 0.012 + grip_to_bottom), FINGER_CLOSED, "press to capture plug"),
        Waypoint(1.5, (px, py, TRAVEL), FINGER_CLOSED, "lift plug out of tray"),
    ]

    # 3. Carry over well, descend and seat plug at bottom
    (wx, wy) = WELL_XY
    r_plan += [
        Waypoint(1.8, (wx, wy, TRAVEL), FINGER_CLOSED, "carry over well mouth"),
        Waypoint(2.0, (wx, wy, WELL_SEAT_Z + 0.012 + grip_to_bottom), FINGER_CLOSED, "lower plug into well"),
        Waypoint(1.0, (wx, wy, WELL_SEAT_Z + 0.012 + grip_to_bottom), FINGER_CLOSED, "thread home into seat"),
        Waypoint(1.8, (wx, wy, TRAVEL), FINGER_CLOSED, "withdraw tool from well"),
        Waypoint(1.8, (tx, ty, TRAVEL), FINGER_CLOSED, "carry tool to cradle"),
        Waypoint(1.2, (tx, ty, th_z + TIP), FINGER_CLOSED, "lower tool"),
        Waypoint(0.8, (tx, ty, th_z + TIP), FINGER_OPEN, "release tool"),
        Waypoint(1.2, (tx, ty, TRAVEL), FINGER_OPEN, "right arm retreat"),
    ]

    # Left Arm: waits for plug installation, then installs ignition coil
    (cx, cy, cz) = COIL_START_POS
    ch_z = cz + BOOT_H + COIL_HEAD_SIZE[2] / 2
    coil_grip_to_bottom = (ch_z - cz) + TIP

    l_plan = [
        Waypoint(16.0, (cx, cy, TRAVEL), FINGER_OPEN, "wait for plug seating"),
        Waypoint(2.0, (cx, cy, TRAVEL), FINGER_OPEN, "move over ignition coil"),
        Waypoint(1.5, (cx, cy, ch_z + TIP), FINGER_OPEN, "lower to coil head"),
        Waypoint(0.8, (cx, cy, ch_z + TIP), FINGER_CLOSED, "grip coil head"),
        Waypoint(1.2, (cx, cy, TRAVEL), FINGER_CLOSED, "lift coil"),
        Waypoint(1.8, (wx, wy, TRAVEL), FINGER_CLOSED, "carry coil over well mouth"),
        Waypoint(2.0, (wx, wy, TARGET_COIL_BOTTOM_Z + coil_grip_to_bottom), FINGER_CLOSED, "insert boot down well"),
        Waypoint(1.0, (wx, wy, TARGET_COIL_BOTTOM_Z + coil_grip_to_bottom), FINGER_CLOSED, "press ear flush"),
        Waypoint(0.8, (wx, wy, TARGET_COIL_BOTTOM_Z + coil_grip_to_bottom), FINGER_OPEN, "release coil"),
        Waypoint(1.5, (wx, wy, TRAVEL), FINGER_OPEN, "lift clear"),
        Waypoint(1.5, (cx, cy, TRAVEL), FINGER_OPEN, "left arm retreat"),
    ]

    return {"left": l_plan, "right": r_plan}


def self_test():
    mk.new_episode()
    # 1. Initial state -> 0.0
    st_init = kit.fake_states()
    assert score(st_init) == 0.0, "initial state must score 0.0"

    # 2. Dropped defect latched -> 0.0
    mk.new_episode()
    mk.MEMORY["plug_dropped_defect"] = True
    assert score(st_init) == 0.0, "dropped plug defect must score 0.0"

    # 3. Plug captured in tray -> 0.25
    mk.new_episode()
    mk.MEMORY["plug_captured"] = True
    assert score(st_init) == 0.25, "captured plug must score 0.25"

    # 4. Plug seated in well, but tool not withdrawn yet -> 0.25
    mk.new_episode()
    st_plug_seated_tool_down = kit.fake_states(moved={
        "spark_plug": (np.array([WELL_XY[0], WELL_XY[1], WELL_SEAT_Z]), np.array([0, 0, 0, 1])),
        "socket_tool": (np.array([WELL_XY[0], WELL_XY[1], WELL_SEAT_Z + 0.010]), np.array([0, 0, 0, 1])),
    })
    assert score(st_plug_seated_tool_down) == 0.25, "plug seated but tool not withdrawn must score 0.25"

    # 5. Plug seated and tool parked in cradle -> 0.50
    mk.new_episode()
    st_seated = kit.fake_states(moved={
        "spark_plug": (np.array([WELL_XY[0], WELL_XY[1], WELL_SEAT_Z]), np.array([0, 0, 0, 1])),
        "socket_tool": (np.array([TOOL_CRADLE_XY[0], TOOL_CRADLE_XY[1], TOOL_CRADLE_Z]), np.array([0, 0, 0, 1])),
    })
    assert score(st_seated) == 0.50, "plug seated and tool clear must score 0.50"

    # 6. Coil entered well mouth -> 0.75
    mk.new_episode()
    mk.MEMORY["coil_entered"] = True
    assert score(st_seated) == 0.75, "coil entered must score 0.75"

    # 7. Coil seated but hands still touching -> 0.75
    global _TEST_TCPS
    _TEST_TCPS = [np.array([WELL_XY[0], WELL_XY[1], TARGET_COIL_BOTTOM_Z + 0.035]), np.array([0.10, -0.20, 0.90])]
    st_touching = kit.fake_states(moved={
        "spark_plug": (np.array([WELL_XY[0], WELL_XY[1], WELL_SEAT_Z]), np.array([0, 0, 0, 1])),
        "socket_tool": (np.array([TOOL_CRADLE_XY[0], TOOL_CRADLE_XY[1], TOOL_CRADLE_Z]), np.array([0, 0, 0, 1])),
        "ignition_coil": (np.array([WELL_XY[0], WELL_XY[1], TARGET_COIL_BOTTOM_Z]), np.array([0, 0, 0, 1])),
    })
    assert score(st_touching) == 0.75, "hands touching coil must not award 1.0 (scores 0.75)"

    # 8. Full success: plug seated, tool clear, coil seated, arms clear -> 1.0
    _TEST_TCPS = [np.array([0.10, 0.20, 0.90]), np.array([0.10, -0.20, 0.90])]
    st_success = kit.fake_states(moved={
        "spark_plug": (np.array([WELL_XY[0], WELL_XY[1], WELL_SEAT_Z]), np.array([0, 0, 0, 1])),
        "socket_tool": (np.array([TOOL_CRADLE_XY[0], TOOL_CRADLE_XY[1], TOOL_CRADLE_Z]), np.array([0, 0, 0, 1])),
        "ignition_coil": (np.array([WELL_XY[0], WELL_XY[1], TARGET_COIL_BOTTOM_Z]), np.array([0, 0, 0, 1])),
    })
    assert score(st_success) == 1.0, "full success must score 1.0"
    assert success(st_success) is True, "success() must return True"

    # 9. Defect overrides complete state -> 0.0
    mk.MEMORY["plug_dropped_defect"] = True
    assert score(st_success) == 0.0, "dropped defect must override complete seating to 0.0"

    # 10. Plug off-axis / tilted -> 0.0
    mk.new_episode()
    st_cocked = kit.fake_states(moved={
        "spark_plug": (np.array([WELL_XY[0] + 0.015, WELL_XY[1], WELL_SEAT_Z]), np.array([0, 0, 0, 1])),
        "socket_tool": (np.array([TOOL_CRADLE_XY[0], TOOL_CRADLE_XY[1], TOOL_CRADLE_Z]), np.array([0, 0, 0, 1])),
    })
    assert score(st_cocked) == 0.0, "plug off-axis in well must score 0.0"

    # 11. Test simulation rules end-to-end on capture
    mk.new_episode()
    st_capture_test = kit.fake_states(moved={
        "socket_tool": (np.array([PLUG_START_POS[0], PLUG_START_POS[1], PLUG_START_POS[2] + 0.012]), np.array([0, 0, 0, 1])),
        "spark_plug": (np.array(PLUG_START_POS), np.array([0, 0, 0, 1])),
    })
    _simulation_rules(st_capture_test)
    assert mk.MEMORY.get("plug_captured") is True, "_simulation_rules must trigger plug_captured when tool cup descends over plug"

    _TEST_TCPS = None
    mk.new_episode()
    return 11


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
