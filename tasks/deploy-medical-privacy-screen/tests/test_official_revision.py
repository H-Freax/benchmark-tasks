"""Offline behavioral review regressions; no simulator/native acceptance.

Only the unavailable robot-hull transport and partkit registration leaf are
replaced. Coverage, support, hinges, release and PrimaryScore.observe are real.
Boundary corpora repeat measured blocked/visible segments to isolate each count.
"""
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import copy
import importlib.util
import json
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'native'))
from coverage_geometry import coverage
from primary_score import PrimaryScore, NAMES


@contextmanager
def robot_transport(near=False):
    """Synthetic 315 hulls/arm; does not claim stock robot or native evidence."""
    analysis = ModuleType('yam_arena.analysis')
    centre = [0.95, 0, 0.5] if near else [-10, -10, -10]
    vertices = np.array([[x,y,z] for x in [-.01,.01] for y in [-.01,.01]
                         for z in [-.01,.01]]) + centre
    analysis.world_hulls = lambda links: [('synthetic_transport', SimpleNamespace(vertices=vertices))] * 315
    arena = ModuleType('yam_arena')
    arena.analysis = analysis
    with patch.dict(sys.modules, {'yam_arena': arena, 'yam_arena.analysis': analysis}):
        yield


def geometry():
    return json.loads((ROOT / 'geometry/geometry.json').read_text())


def initial_row(g, k=0):
    bodies = {}
    for p in g['parts']:
        if p['name'] not in NAMES:
            continue
        pos = list(p['pos'])
        pos[2] += g['floor_surface_z_m']  # Static floor-supported reset geometry.
        bodies[p['name']] = dict(pose_xyzw=pos + p['quat_xyzw'], native_velocity=[0.] * 6)
    return dict(physics_step=k * 48, time_s=k / 20, rigid_bodies=bodies,
                robots={s: dict(body_names=['transport'], body_link_pose_w=[[0,0,0,0,0,0,1]])
                        for s in ['left','right']})


def final_window(g, row=None, n=21):
    scorer = PrimaryScore()
    template = initial_row(g) if row is None else row
    rows = []
    for k in range(n):
        r = copy.deepcopy(template)
        r.update(physics_step=k * 48, time_s=k / 20)
        rows.append(scorer.observe(r, g))
    return scorer, rows


def boundary_geometry(blocked):
    g = geometry()
    poses = [initial_row(g)['rigid_bodies'][n]['pose_xyzw'] for n in NAMES]
    eye = g['doorway_sight_points_m'][0]
    classified = [(t, coverage(poses, [eye], [t], g)) for t in g['target_samples_m']]
    hidden = next(t for t,c in classified if c == 1.)
    visible = next(t for t,c in classified if c == 0.)
    g['doorway_sight_points_m'] = [eye]
    g['target_samples_m'] = [hidden] * blocked + [visible] * (567 - blocked)
    return g


class ScoringRevision(unittest.TestCase):
    def test_actual_initial_layout_has_no_credit_after_complete_window(self):
        g = geometry()
        poses = [initial_row(g)['rigid_bodies'][n]['pose_xyzw'] for n in NAMES]
        self.assertAlmostEqual(coverage(poses, g['doorway_sight_points_m'], g['target_samples_m'], g), 486 / 567)
        with robot_transport():
            scorer, rows = final_window(g)
        self.assertTrue(all(r['valid'] and r['supported'] and r['hands_clear'] and r['hinge_integrity'] for r in rows))
        self.assertEqual(len(rows), 21)
        self.assertEqual(scorer.value, 0., rows[-1])

    def test_exact_510_511_561_562_boundaries_use_real_coverage(self):
        for count, expected in [(510,0.), (511,.5), (561,.5), (562,1.)]:
            with self.subTest(blocked=count), robot_transport():
                scorer, rows = final_window(boundary_geometry(count))
                self.assertTrue(all(r['valid'] for r in rows), rows[-1])
                self.assertAlmostEqual(rows[-1]['coverage_fraction'], count / 567)
                self.assertTrue(all(r['score'] == 0. for r in rows[:20]))
                self.assertEqual(scorer.value, expected, rows[-1])
        # A real articulated intermediate on the unchanged 3x189 grid: rotate
        # only the left wing about its existing hinge, keeping all anchors.
        g = geometry()
        row = initial_row(g)
        joint = g['joints'][0]
        parent = row['rigid_bodies'][joint['parent']]['pose_xyzw']
        anchor = np.array(parent[:3]) + Rotation.from_quat(parent[3:]).as_matrix() @ joint['parent_local_pos']
        rotation = Rotation.from_euler('z', 190, degrees=True)
        row['rigid_bodies'][joint['child']]['pose_xyzw'] = (anchor - rotation.as_matrix() @ joint['child_local_pos']).tolist() + rotation.as_quat().tolist()
        with robot_transport():
            scorer, rows = final_window(g, row)
        self.assertAlmostEqual(rows[-1]['coverage_fraction'], 522 / 567)
        self.assertEqual(scorer.value, .5, rows[-1])

    def test_freshness_invalid_support_release_gates_remain_required(self):
        g = boundary_geometry(562)
        with robot_transport():
            scorer, rows = final_window(g)
            self.assertEqual(scorer.value, 1.)
            duplicate = initial_row(g, 20)
            self.assertFalse(scorer.observe(duplicate, g)['valid'])
            self.assertEqual(scorer.value, 0.)
            scorer, _ = final_window(g)
            self.assertFalse(scorer.observe(initial_row(g, 22), g)['valid'])
            bad = initial_row(g)
            bad['rigid_bodies']['left_panel']['native_velocity'][0] = float('nan')
            self.assertFalse(PrimaryScore().observe(bad, g)['valid'])
            lifted = initial_row(g)
            for b in lifted['rigid_bodies'].values():
                b['pose_xyzw'][2] += .02
            scorer, rows = final_window(g, lifted)
            self.assertFalse(rows[-1]['supported'])
            self.assertEqual(scorer.value, 0.)
        with robot_transport(near=True):
            scorer, rows = final_window(g)
            self.assertTrue(rows[-1]['valid'], rows[-1])
            self.assertFalse(rows[-1]['hands_clear'])
            self.assertEqual(scorer.value, 0.)


class VisibleObservationMarks(unittest.TestCase):
    def test_registered_visible_marks_match_all_actual_sight_points(self):
        # Execute the real task's load/registration prefix with an inert partkit
        # leaf; native_geometry.objects itself is the actual production function.
        @dataclass(frozen=True)
        class Part:
            name: str
            pos: tuple
            size: tuple
            mass: float
            color: tuple
            quat: tuple
            fixed: bool
            damping: float
        pk = ModuleType('partkit')
        pk.Part = Part
        pk._arena_object = lambda spec: spec
        pk._patch_scene = lambda mod: None
        pk._Finder = SimpleNamespace(targets={})
        arena = ModuleType('yam_arena')
        arena.station = SimpleNamespace(BASES={})
        with patch.dict(sys.modules, {'partkit': pk, 'yam_arena': arena}):
            spec = importlib.util.spec_from_file_location('revision_geometry', ROOT / 'native/native_geometry.py')
            ng = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = ng
            try:
                spec.loader.exec_module(ng)
                source = (ROOT / 'native/task.py').read_text().split('TRACE=')[0]
                with patch.dict(sys.modules, {'native_geometry': ng}):
                    env = {'__file__': str(ROOT / 'native/task.py')}
                    exec(compile(source, str(ROOT / 'native/task.py'), 'exec'), env)
                marks = [o for o in env['OBJECTS'] if o.name.startswith('doorway_view_')]
                self.assertEqual(len(marks), 3)
                original = geometry()
                self.assertEqual(env['G']['doorway_sight_points_m'], original['doorway_sight_points_m'])
                self.assertEqual(env['G']['parts'][:len(original['parts'])], original['parts'])
                for i,(mark,eye) in enumerate(zip(marks, original['doorway_sight_points_m'], strict=True), 1):
                    self.assertEqual(mark.name, f'doorway_view_{i}')
                    self.assertEqual(list(mark.pos), eye)
                    self.assertTrue(mark.fixed)
                    shapes = mark.native_spec['geoms']
                    self.assertTrue(shapes)
                    self.assertTrue(all(not s['collision_enabled'] for s in shapes))
                    self.assertTrue(all(s['color'] == [1., .65, .05] for s in shapes))
                    centres = np.array([s['pos'] for s in shapes])
                    np.testing.assert_allclose(centres.mean(0), [0,0,0], atol=1e-12)
                    self.assertTrue(all(s['type'] == 'box' and min(s['size']) > 0 for s in shapes))
                    self.assertGreater(max(np.linalg.norm(c) for c in centres), .02)
            finally:
                sys.modules.pop(spec.name, None)


if __name__ == '__main__':
    unittest.main(verbosity=2)
