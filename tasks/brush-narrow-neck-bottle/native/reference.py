"""Scripted reference for the bottle brushing task, played by the ``yam_scripted`` policy.

Left arm: hold the bottle by its body. Right arm: pick the brush up by its handle, stand it upright, lower
the head through the neck, scrub the bottom and then the shoulder with small circles, lift the brush out and
lay it back on its tray; the left hand then lets the bottle go.

Only ordinary joint targets are sent. While the brush is near or inside the bottle the right arm's target is
set from the observed brush and bottle poses, so the brush axis follows the neck wherever the held bottle
actually is. The run stops early only on a real hazard: the bottle tipping, the handle grasp lost, or the
steel stem pressing on the glass harder than ``STEM_FORCE_LIMIT_N``. Ordinary contact between the soft head
and the glass is expected and is not a stop condition.

``BOTTLE_DEMO`` selects a deliberately incomplete variant for checking the scorer:
``bottom_only`` (shoulder skipped) or ``leave_brush_in`` (brush left standing in the bottle).
"""
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, READY
from yam_arena.demo import grip_rotation

VARIANTS = ("full", "bottom_only", "leave_brush_in")
HZ = 20.0
EZ = np.array([0.0, 0.0, 1.0])
EX = np.array([1.0, 0.0, 0.0])

# Hand orientations (TCP frame: +z approach axis, fingers close along +-x).
R_DOWN = grip_rotation(0.0)                                                # straight down, closing along y
R_FWD = Rotation.from_euler("y", -90, degrees=True).as_matrix() @ R_DOWN   # approach +x: a held brush hangs down
_T = math.radians(15.0)                                                    # left hand comes in from +y, 15 deg down
R_HOLD = np.column_stack([[-1.0, 0.0, 0.0], [0.0, math.sin(_T), -math.cos(_T)], [0.0, -math.cos(_T), -math.sin(_T)]])

# Nominal handle grasp: brush pose in the hand frame when the handle centre is taken from above.
GRASP_Z = 0.22
G_NOMINAL = np.eye(4)
G_NOMINAL[:3, :3] = R_DOWN.T @ Rotation.from_euler("y", 90, degrees=True).as_matrix()
G_NOMINAL[:3, 3] = R_DOWN.T @ np.array([-GRASP_Z, 0.0, 0.0])

# Brush tip heights in the bottle frame (inner bottom at 4 mm, shoulder cone 94-114 mm, mouth at 139 mm) and
# circle radii of the brush axis about the bottle axis. Each puts the 42 mm head 1.5 to 2 mm into the glass,
# which the soft head takes up at 0.2 to 0.3 N per contact point. Under the shoulder the head's top edge has to
# be within about a millimetre of the right height, so the tip height is held by its own feedback (KZ): the
# soft wrist otherwise lets the head ride 2 mm up and down as the cone pushes back around the circle.
Z_ABOVE = 0.151
Z_BOTTOM = 0.0025
Z_SHOULDER_LOW = 0.0602     # head top at 100.2 mm, under the lower ring of shoulder marks (99 mm)
Z_SHOULDER_HIGH = 0.0702    # head top at 110.2 mm, under the upper ring (109 mm)
RHO_BOTTOM = 0.0070
RHO_SHOULDER_LOW = 0.0080
RHO_SHOULDER_HIGH = 0.0043

KI = 0.12                    # per step, on the right arm's joint error while placing the brush precisely
KZ = 0.30                    # per step, on the brush tip's height error while scrubbing
Z_TRIM_LIMIT = 0.005         # m
I_LIMIT = 0.20               # rad
STEM_FORCE_LIMIT_N = 15.0
BOTTLE_TILT_LIMIT = math.radians(25.0)


def _T4(pose7):
    a = np.asarray(pose7, float)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(a[3:]).as_matrix()
    T[:3, 3] = a[:3]
    return T


def _smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def _slerp(R0, R1, s):
    return Rotation.from_rotvec(Rotation.from_matrix(R1 @ R0.T).as_rotvec() * s).as_matrix() @ R0


def _align(a, b):
    """Smallest rotation taking unit vector a onto unit vector b."""
    axis = np.cross(a, b)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    return Rotation.from_rotvec(axis / n * math.atan2(n, float(a @ b))).as_matrix()


def _circle(rho, turns, ramp, z):
    """Tip path in the bottle frame: spiral out, ``turns`` circles of radius ``rho``, spiral in, at height z."""

    def fn(s):
        r = rho * min(1.0, s / ramp, (1.0 - s) / ramp)
        a = 2 * math.pi * turns * s
        return np.array([r * math.cos(a), r * math.sin(a), z])

    return fn


def _line(z0, z1):
    return lambda s: np.array([0.0, 0.0, z0 + (z1 - z0) * _smooth(s)])


def script(variant="full"):
    """The timeline: (name, steps, left, right, (left grip, right grip), options). Grip 1 is open, 0 closed.

    Arm entries: None holds; ("hand", p, R) moves the hand; ("shift", dp) moves it by dp; ("bottle", fn)
    puts the brush tip at fn(s) in the observed bottle frame with the brush axis on the bottle axis;
    ("lay", p) brings the tip to world point p with the brush horizontal along +x.
    """
    if variant not in VARIANTS:
        raise ValueError(f"BOTTLE_DEMO must be one of {VARIANTS}, got {variant!r}")
    hover_l, hold_l = (0.30, 0.16, 0.83), (0.30, 0.0, 0.805)
    hover_r, grasp_r = (0.34, -0.18, 0.89), (0.34, -0.18, 0.772)
    carry_r, high_r = (0.25, -0.18, 0.90), (0.30, -0.10, 1.12)
    o, c = 1.0, 0.0
    steps = [
        ("approach", 40, ("hand", hover_l, R_HOLD), ("hand", hover_r, R_DOWN), (o, o), {}),
        ("reach", 50, ("hand", hold_l, R_HOLD), ("hand", grasp_r, R_DOWN), (o, o), {}),
        ("close", 20, None, None, (c, c), {"gate": "grasp"}),
        ("lift_brush", 50, None, ("hand", carry_r, R_DOWN), (c, c), {}),
        ("stand_brush_up", 80, None, ("hand", high_r, R_FWD), (c, c), {}),
        ("above_neck", 60, None, ("bottle", _line(Z_ABOVE, Z_ABOVE)), (c, c), {"blend": True, "gate": "aligned"}),
        ("insert", 110, None, ("bottle", _line(Z_ABOVE, 0.010)), (c, c), {}),
        ("touch_bottom", 20, None, ("bottle", _line(0.010, Z_BOTTOM)), (c, c), {}),
        ("scrub_bottom", 190, None, ("bottle", _circle(RHO_BOTTOM, 3, 20 / 190, Z_BOTTOM)), (c, c), {"hold_z": True}),
    ]
    if variant == "bottom_only":
        steps.append(("withdraw", 130, None, ("bottle", _line(Z_BOTTOM, Z_ABOVE)), (c, c), {}))
    else:
        steps += [
            ("raise_to_shoulder", 50, None, ("bottle", _line(Z_BOTTOM, Z_SHOULDER_LOW)), (c, c), {}),
            ("scrub_lower_shoulder", 440, None,
             ("bottle", _circle(RHO_SHOULDER_LOW, 5, 20 / 440, Z_SHOULDER_LOW)), (c, c), {"hold_z": True}),
            ("raise_to_upper_shoulder", 20, None, ("bottle", _line(Z_SHOULDER_LOW, Z_SHOULDER_HIGH)), (c, c), {}),
            ("scrub_upper_shoulder", 520, None,
             ("bottle", _circle(RHO_SHOULDER_HIGH, 6, 20 / 520, Z_SHOULDER_HIGH)), (c, c), {"hold_z": True}),
        ]
        if variant == "leave_brush_in":
            return steps + [
                ("let_go_of_brush", 20, None, None, (c, o), {"free": True}),
                ("right_hand_back", 40, None, ("shift", (-0.10, 0.0, 0.0)), (c, o), {"free": True}),
                ("release_bottle", 20, None, None, (o, o), {"free": True}),
                ("left_hand_away", 40, ("hand", hover_l, R_HOLD), None, (o, o), {"free": True}),
                ("rest", 80, None, None, (o, o), {"free": True}),
            ]
        steps.append(("withdraw", 100, None, ("bottle", _line(Z_SHOULDER_HIGH, Z_ABOVE)), (c, c), {}))
    return steps + [
        ("move_aside", 40, None, ("hand", (0.30, -0.12, 1.12), R_FWD), (c, c), {}),
        ("release_bottle", 20, None, None, (o, c), {}),
        ("turn_brush_flat", 80, ("hand", hover_l, R_HOLD), ("hand", carry_r, R_DOWN), (o, c), {}),
        ("lay_brush_down", 60, None, ("lay", (0.12, -0.18, 0.773)), (o, c), {"blend": True}),
        ("let_go_of_brush", 20, None, None, (o, o), {"free": True}),
        ("right_hand_away", 40, None, ("hand", hover_r, R_DOWN), (o, o), {"free": True}),
        ("rest", 80, None, None, (o, o), {"free": True}),
    ]


class Reference:
    """Steps through ``script`` once per control step; ``obs`` is what the task observed after the last step."""

    def __init__(self, variant="full", log=None, log_step=None):
        self.steps = script(variant)
        self.variant = variant
        self.log = log
        self.log_step = log_step
        self.i = 0                  # phase index
        self.k = 0                  # step inside the phase
        self.wait = 0               # extra steps spent on a gate
        self.streak = 0
        self.count = 0
        self.q_des = {a: np.array(READY, float) for a in ("left", "right")}
        self.integral = np.zeros(6)
        self.grip = {"left": 1.0, "right": 1.0}
        self.grip_start = dict(self.grip)
        self.hand_des = {}          # last desired hand pose per arm (4x4)
        self.start = {}             # desired hand pose per arm at the start of the phase
        self.G = G_NOMINAL.copy()   # brush pose in the right hand frame (measured while held)
        self.z_trim = 0.0           # added to the wanted tip height in the bottle frame (see KZ)
        self.lost = 0
        self.failure = None
        self.failed_steps = 0
        self.done = False
        self.stop_requested = False
        self.worst_ik = (0.0, 0.0)
        self.notes = {}

    # -- targets ----------------------------------------------------------------------------------------------

    def _phase_start(self):
        self.start = {a: T.copy() for a, T in self.hand_des.items()}
        self.grip_start = dict(self.grip)

    def _hand_target(self, side, spec, s, obs, options):
        start = self.start[side]
        if spec is None:
            return start
        kind = spec[0]
        w = _smooth(s)
        out = np.eye(4)
        if kind == "hand":
            out[:3, :3] = _slerp(start[:3, :3], np.asarray(spec[2]), w)
            out[:3, 3] = start[:3, 3] + w * (np.asarray(spec[1], float) - start[:3, 3])
            return out
        if kind == "shift":
            out[:3, :3] = start[:3, :3]
            out[:3, 3] = start[:3, 3] + w * np.asarray(spec[1], float)
            return out
        # Brush-referenced targets: the tip position and the hand rotation that puts the brush axis where wanted.
        Gp, GR = self.G[:3, 3], self.G[:3, :3]
        if kind == "bottle":
            B = obs["bottle"]
            want = spec[1](s)
            if options.get("hold_z"):
                actual_z = float((np.linalg.inv(B) @ obs["brush"])[2, 3])
                self.z_trim = float(np.clip(self.z_trim + KZ * (want[2] - actual_z), -Z_TRIM_LIMIT, Z_TRIM_LIMIT))
            else:
                self.z_trim *= 0.8
            tip = B[:3, 3] + B[:3, :3] @ (want + np.array([0.0, 0.0, self.z_trim]))
            R = _align(R_FWD @ GR @ EZ, B[:3, :3] @ EZ) @ R_FWD
        elif kind == "lay":
            tip = np.asarray(spec[1], float)
            R = _align(R_DOWN @ GR @ EZ, EX) @ R_DOWN
        else:
            raise ValueError(kind)
        if options.get("blend"):
            tip = (start[:3, 3] + start[:3, :3] @ Gp) * (1 - w) + tip * w
            R = _slerp(start[:3, :3], R, w)
        out[:3, :3] = R
        out[:3, 3] = tip - R @ Gp
        return out

    # -- gates and hazards ------------------------------------------------------------------------------------

    def _gate(self, name, obs):
        if name == "grasp":
            ok = obs["holding"]["left"] and obs["holding"]["right"]
            limit, must = 40, True
        elif name == "aligned":
            rel = np.linalg.inv(obs["bottle"]) @ obs["brush"]
            lateral = float(np.linalg.norm(rel[:2, 3]))
            tilt = math.acos(min(1.0, max(-1.0, float(rel[2, 2]))))
            self.notes["alignment"] = dict(lateral_m=lateral, tilt_rad=tilt, waited=self.wait)
            ok = lateral <= 0.0012 and tilt <= math.radians(1.0)
            limit, must = 120, False
        else:
            raise ValueError(name)
        self.streak = self.streak + 1 if ok else 0
        if self.streak >= 3:
            return True
        self.wait += 1
        if self.wait > limit:
            if must:
                raise RuntimeError(f"{name} not reached")
            return True
        return False

    def _hazards(self, obs, holding_expected):
        tilt = math.acos(min(1.0, max(-1.0, float(obs["bottle"][2, 2]))))
        if tilt > BOTTLE_TILT_LIMIT:
            raise RuntimeError(f"bottle tipped {math.degrees(tilt):.0f} deg")
        if obs["stem_force"] > STEM_FORCE_LIMIT_N:
            raise RuntimeError(f"stem pressing on the glass with {obs['stem_force']:.1f} N")
        if holding_expected:
            self.lost = 0 if obs["holding"]["right"] else self.lost + 1
            if self.lost > 10:
                raise RuntimeError("handle grasp lost")

    # -- one control step -------------------------------------------------------------------------------------

    def step(self, obs):
        """Joint targets and grips for both arms for the next control step."""
        self.count += 1
        if not self.hand_des:
            self.hand_des = {a: obs["hand"][a].copy() for a in ("left", "right")}
            self._phase_start()
        try:
            if self.failure:
                raise RuntimeError(self.failure)
            if self.done:
                return self._command(obs, integrate=False)
            name, n, left, right, grips, options = self.steps[self.i]
            held = self.i > 2 and not options.get("free")
            self._hazards(obs, held)
            if held and obs["holding"]["right"]:
                M = np.linalg.inv(obs["hand"]["right"]) @ obs["brush"]
                self.G[:3, 3] += 0.25 * (M[:3, 3] - self.G[:3, 3])
                self.G[:3, :3] = _slerp(self.G[:3, :3], M[:3, :3], 0.25)
            self.k = min(self.k + 1, n)
            s = self.k / n
            for side, spec in (("left", left), ("right", right)):
                self.hand_des[side] = self._hand_target(side, spec, s, obs, options)
                self.grip[side] = self.grip_start[side] + (grips[0 if side == "left" else 1] - self.grip_start[side]) * s
            precise = right is not None and right[0] in ("bottle", "lay")
            out = self._command(obs, integrate=precise)
            if self.log_step:
                rel = np.linalg.inv(obs["bottle"]) @ obs["brush"]
                want = right[1](s).tolist() if right is not None and right[0] == "bottle" else None
                self.log_step(dict(control=self.count, phase=name, tip_in_bottle=rel[:3, 3].tolist(), wanted=want,
                                   axis_rad=math.acos(min(1.0, max(-1.0, float(rel[2, 2])))),
                                   integral=self.integral.tolist(), z_trim=self.z_trim, grasp=self.G[:3, 3].tolist(),
                                   grasp_rot=Rotation.from_matrix(self.G[:3, :3] @ G_NOMINAL[:3, :3].T).as_rotvec().tolist(),
                                   stem_force=obs["stem_force"], holding=obs["holding"]))
            if self.k >= n and (not options.get("gate") or self._gate(options["gate"], obs)):
                if self.log:
                    self.log(dict(event="phase_done", phase=name, control=self.count, waited=self.wait, notes=self.notes))
                self.i += 1
                self.k = self.wait = self.streak = 0
                self._phase_start()
                if self.i >= len(self.steps):
                    self.done = self.stop_requested = True
            return out
        except RuntimeError as error:
            if self.failure is None:
                self.failure = str(error)
                if self.log:
                    self.log(dict(event="stopped", reason=self.failure, control=self.count, phase=self.steps[min(self.i, len(self.steps) - 1)][0]))
            self.failed_steps += 1
            if self.failed_steps >= 10:
                self.stop_requested = True
            return {a: (np.clip(self.q_des[a] + (self.integral if a == "right" else 0), ik.LOWER, ik.UPPER), self.grip[a]) for a in ("left", "right")}

    def _command(self, obs, integrate):
        out = {}
        for side in ("left", "right"):
            T = self.hand_des[side]
            q, pe, re = ik.solve(T[:3, 3], ik.matrix_to_quat_xyzw(T[:3, :3]), self.q_des[side],
                                 base_pos=station.BASES[side], iters=100, damping=0.03, tol=1e-4)
            self.worst_ik = (max(self.worst_ik[0], pe), max(self.worst_ik[1], re))
            if pe > 0.003 or re > 0.03:
                raise RuntimeError(f"{side} hand target out of reach by {pe * 1000:.1f} mm / {math.degrees(re):.1f} deg")
            self.q_des[side] = q
            if side == "right":
                if integrate:
                    self.integral = np.clip(self.integral + KI * (q - obs["q"]["right"]), -I_LIMIT, I_LIMIT)
                q = q + self.integral
            out[side] = (np.clip(q, ik.LOWER, ik.UPPER), self.grip[side])
        return out


# -- reading the task's observation ------------------------------------------------------------------------------


def observation(row):
    """What the reference uses from the task's per-step observation row."""
    from pad_contacts import two_pad

    hand, q = {}, {}
    for side in ("left", "right"):
        d = row["robots"][side]
        T = _T4(d["links"][d["body_names"].index("gripper")])
        T[:3, 3] += T[:3, :3] @ np.array([0.0, 0.0, 0.134])
        hand[side] = T
        q[side] = np.array([d["q"][d["joint_names"].index(n)] for n in ARM_JOINTS], float)
    brush = _T4(row["brush_pose"])
    inv = np.linalg.inv(brush)
    stem = 0.0
    for c in row["native_contacts"]:
        local = inv[:3, :3] @ np.asarray(c["world_point_m"], float) + inv[:3, 3]
        if local[2] > 0.045:
            stem += abs(float(c["normal_force_N"]))
    pads = row["pad_contacts"]
    return dict(bottle=_T4(row["bottle_pose"]), brush=brush, hand=hand, q=q, stem_force=stem,
                holding=dict(left=two_pad(pads, "feeding_bottle", "left"), right=two_pad(pads, "bottle_brush", "right")))


def install(scene, variant="full"):
    """Hook the reference into yam_scripted for this episode and return the demo() plan that selects it."""
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import ArmPlan, Waypoint
    from yam_arena.scripted import YamScriptedPolicy

    trace = scene.TRACE

    def log(entry):
        print("BOTTLE_REFERENCE " + json.dumps(entry), flush=True)
        if trace is not None:
            trace.mkdir(parents=True, exist_ok=True)
            with (trace / "reference.jsonl").open("a") as f:
                f.write(json.dumps(entry) + "\n")

    def log_step(entry):
        if trace is not None:
            trace.mkdir(parents=True, exist_ok=True)
            with (trace / "reference-steps.jsonl").open("a") as f:
                f.write(json.dumps(entry) + "\n")

    state = dict(control=Reference(variant, log, log_step if trace is not None else None), t=None, out=None)
    ArmPlan._bottle_reference = state
    if not getattr(ArmPlan, "_bottle_patched", False):
        original_action7, original_act = ArmPlan.action7, YamScriptedPolicy.act

        def action7(plan, t):
            if not (plan.waypoints and plan.waypoints[0].label.startswith("bottle_reference_")):
                return original_action7(plan, t)
            st = ArmPlan._bottle_reference
            if st["t"] != t:
                st["t"] = t
                if scene.LATEST is None:
                    raise RuntimeError("no observation before the first action")
                st["out"] = st["control"].step(observation(scene.LATEST))
            q, grip = st["out"][plan.waypoints[0].label.rsplit("_", 1)[1]]
            return np.r_[q, grip], "bottle_reference"

        def act(self, obs):
            result = original_act(self, obs)
            if ArmPlan._bottle_reference["control"].stop_requested:
                for action in result.actions:
                    action.meta["request_stop"] = True
            return result

        ArmPlan.action7, YamScriptedPolicy.act = action7, act
        ArmPlan._bottle_patched = True
    return {a: [Waypoint(600.0, (0.2, 0.0, 0.8), FINGER_OPEN, "bottle_reference_" + a)] for a in ("left", "right")}


# -- offline replay (no simulator) ---------------------------------------------------------------------------------


def dry_run(G, variant="full", bottle_pose=(0.30, 0.0, 0.7475, 0.0, 0.0, 0.0, 1.0)):
    """Replay the reference on the kinematics alone: the arms reach their joint targets exactly, the brush
    rides rigidly in the nominal handle grasp and the bottle does not move. Returns the plan length, the worst
    IK error and the film coverage the brush path alone would give (contact assumed while scrubbing)."""
    from cleaning import Film

    parts = {p["name"]: p for p in G["parts"]}
    bottle = _T4(bottle_pose)
    brush = _T4(parts["bottle_brush"]["pos"] + parts["bottle_brush"]["quat_xyzw"])
    ref = Reference(variant)
    film = Film(G)
    q = {a: np.array(READY, float) for a in ("left", "right")}
    attached = False
    names = [s[0] for s in ref.steps]
    limits_ok = True
    while not ref.done and ref.count < 6000:
        hand = {}
        for a in ("left", "right"):
            p, R = ik.tcp_pose(q[a], station.BASES[a])
            hand[a] = np.eye(4)
            hand[a][:3, :3], hand[a][:3, 3] = R, p
        phase = names[ref.i]
        if phase == "lift_brush":
            attached = True
        if phase in ("let_go_of_brush",):
            attached = False
        if attached:
            brush = hand["right"] @ G_NOMINAL
        holding = dict(left=names.index("close") <= ref.i < names.index("release_bottle"), right=attached or phase == "close")
        out = ref.step(dict(bottle=bottle, brush=brush, hand=hand, q=q, stem_force=0.0, holding=holding))
        if ref.failure:
            raise RuntimeError(f"{variant}: {ref.failure}")
        for a in ("left", "right"):
            limits_ok &= bool((out[a][0] >= ik.LOWER - 1e-9).all() and (out[a][0] <= ik.UPPER + 1e-9).all())
            q[a] = out[a][0]
        scrubbing = phase.startswith("scrub")
        contact = [dict(normal_force_N=1.0, world_point_m=(brush[:3, 3] + brush[:3, :3] @ np.array([0, 0, 0.02])).tolist())]
        pose7 = lambda T: T[:3, 3].tolist() + Rotation.from_matrix(T[:3, :3]).as_quat().tolist()  # noqa: E731
        film.update(dict(serial=ref.count, bottle_pose=pose7(bottle), brush_pose=pose7(brush),
                         native_contacts=contact if scrubbing else []))
    return dict(steps=ref.count, seconds=ref.count / HZ, worst_ik_mm=ref.worst_ik[0] * 1000,
                worst_ik_deg=math.degrees(ref.worst_ik[1]), within_joint_limits=limits_ok, coverage=film.coverage())
