"""CPU-safe task bridge. Native imports happen only after official Kit bootstrap.

Installs deferred USD spawning, filtered door-to-pad contact-force readback,
and 6-DoF waypoint interpolation. Reads physical poses/forces, never writes door state.
"""
from dataclasses import dataclass
import importlib.abc
import importlib.machinery
import sys
import numpy as np
from yam_arena.task_file import Box


@dataclass(frozen=True)
class USDPart(Box):
    usd_path: str = ''
    origin: tuple = (0.,0.,0.)
    kinematic: bool = False


@dataclass
class DoorWaypoint:
    duration: float
    pos: tuple
    finger: float
    label: str = ''
    quat_xyzw: tuple = (0.,0.,0.,1.)
    lift_off_waypoint: bool = True


def array(value):
    value=getattr(value,'torch',value)
    if hasattr(value,'detach'):
        return value.detach().cpu().numpy().astype(float)
    if hasattr(value,'numpy'):
        return value.numpy().astype(float)
    return np.asarray(value,float)


def upward_support_force(value):
    a=np.asarray(value,float)
    if not a.size or a.shape[-1]!=3 or not np.isfinite(a).all():
        raise ValueError('invalid support contact matrix')
    # PhysX reports force on sensor body (door); preserve sign.
    return max(0.,float(a.reshape(-1,3)[:,2].sum()))


def _is_task(emb):
    return emb.task is not None and getattr(emb.task.module,'LIFT_OFF_CABINET_TASK',False)


def sample_native(emb):
    from yam_arena.analysis import link_hulls
    from yam_arena.urdf import quat_xyzw_to_matrix
    scene=emb._base.scene
    env_origin=array(scene.env_origins)[0]
    door=scene['door']
    pose=array(door.data.root_link_pose_w)[0]
    vel=array(door.data.root_com_vel_w)[0]
    view=scene['door_pad_contacts'].contact_view
    if view.filter_count!=1:
        raise RuntimeError(f'expected one live door-pad filter; got {view.filter_count}')
    pad_force=upward_support_force(array(view.get_contact_force_matrix(dt=emb._base.physics_dt)))
    g=emb.task.module.g
    vertices=g.DOOR_VERTICES@quat_xyzw_to_matrix(pose[3:]).T+pose[:3]
    low,high=vertices.min(0),vertices.max(0)
    distances=[]
    hulls=link_hulls()
    for key in ('left_robot','right_robot'):
        robot=scene[key]
        poses=array(robot.data.body_link_pose_w)[0]
        if (poses.shape!=(len(robot.body_names),7) or not np.isfinite(poses).all()
                or not set(hulls).issubset(set(robot.body_names))
                or np.any(abs(np.linalg.norm(poses[:,3:],axis=1)-1)>1e-5)):
            raise ValueError(f'incomplete robot transforms: {key}')
        for name,lp in zip(robot.body_names,poses,strict=True):
            for h in hulls.get(name,()):
                pts=h.vertices@quat_xyzw_to_matrix(lp[3:]).T+lp[:3]
                gap=np.maximum(np.maximum(low-pts.max(0),pts.min(0)-high),0.)
                distances.append(float(np.linalg.norm(gap)))
    if not distances:
        raise RuntimeError('no URDF hulls for release measurement')
    # Static table is not a rigid filter target. Table-only partial support is
    # measured geometrically plus settled velocity/release in the scorer.
    return dict(time=emb._t,origin=pose[:3]-env_origin,quat=pose[3:],lin_vel=vel[:3],ang_vel=vel[3:],
                robot_clearance=min(distances),pad_up_force=pad_force,table_up_force=0.,measurement_valid=True)


def patch_scene(mod):
    if getattr(mod,'_lift_off_cabinet_bridge_v1',False):
        return
    mod._lift_off_cabinet_bridge_v1=True
    object0,build0=mod.task_object,mod.build_task_environment
    def task_object(spec):
        if not isinstance(spec,USDPart):
            return object0(spec)
        return mod.Object(name=spec.name,usd_path=spec.usd_path,
                          object_type=mod.ObjectType.RIGID if spec.free or spec.kinematic else mod.ObjectType.BASE,
                          initial_pose=mod.Pose(position_xyz=tuple(float(v) for v in spec.origin)))
    def build_task_environment(embodiment,task,control_hz=None):
        arena=build0(embodiment,task,control_hz)
        if not getattr(task.module,'LIFT_OFF_CABINET_TASK',False):
            return arena
        from isaaclab.sensors import ContactSensorCfg
        cfg0=arena.env_cfg_callback
        def configure(cfg):
            cfg=cfg0(cfg)
            cfg.scene.door_pad_contacts=ContactSensorCfg(
                prim_path='{ENV_REGEX_NS}/door',filter_prim_paths_expr=['{ENV_REGEX_NS}/landing_pad'],
                history_length=0,update_period=0.,max_contact_data_count_per_prim=256)
            return cfg
        arena.env_cfg_callback=configure
        return arena
    mod.task_object,mod.build_task_environment=task_object,build_task_environment


class LiftOffSceneFinder(importlib.abc.MetaPathFinder):
    lift_off_scene_hook=True
    def find_spec(self,fullname,path=None,target=None):
        if fullname!='yam_arena.scene':
            return None
        spec=importlib.machinery.PathFinder.find_spec(fullname,path)
        if spec is None or spec.loader is None:
            return None
        inner=spec.loader
        class Loader(importlib.abc.Loader):
            def create_module(self,spec):
                return inner.create_module(spec)
            def exec_module(self,module):
                inner.exec_module(module)
                patch_scene(module)
        spec.loader=Loader()
        return spec


def install():
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from yam_arena.demo import ArmPlan
    from yam_arena import ik
    from yam_arena.urdf import quat_xyzw_to_matrix
    if not getattr(ArmPlan,'_lift_off_cabinet_bridge_v1',False):
        target0=ArmPlan.target
        def target(self,t):
            if not self.waypoints or not getattr(self.waypoints[0],'lift_off_waypoint',False):
                return target0(self,t)
            prev,rot,finger=self.start_pos,self.start_rot,self.finger_start
            for w in self.waypoints:
                goal=np.asarray(w.pos,float)
                goal_rot=quat_xyzw_to_matrix(np.asarray(w.quat_xyzw,float))
                if t<w.duration:
                    s=ik.smoothstep(t/w.duration)
                    return prev+s*(goal-prev),ik.slerp_matrix(rot,goal_rot,s),finger+s*(w.finger-finger),w.label
                t-=w.duration
                prev,rot,finger=goal,goal_rot,w.finger
            return prev,rot,finger,'done'
        ArmPlan.target=target
        ArmPlan._lift_off_cabinet_bridge_v1=True
    cls=YamArenaEmbodiment
    if not getattr(cls,'_lift_off_cabinet_bridge_v1',False):
        reset0,step0=cls.reset,cls._task_step
        def reset(self,*a,**kw):
            if _is_task(self):
                if self.object_jitter!=0 or abs(self.info.control_hz-20)>1e-8:
                    raise ValueError('lift-off cabinet requires object_jitter=0, control_hz=20')
                self.task.module.SCORER.reset()
            obs=reset0(self,*a,**kw)
            if _is_task(self):
                reading=sample_native(self)
                self.task.module.SCORER.update(reading)
                if np.linalg.norm(reading['origin']-self.task.module.g.START)>.005:
                    raise RuntimeError(f"door did not remain on hinge shoulders at reset: {reading['origin']}")
            return obs
        def _task_step(self,*a,**kw):
            if _is_task(self):
                try:
                    self.task.module.SCORER.update(sample_native(self))
                except Exception:
                    self.task.module.SCORER.update({})
                    raise
            result=step0(self,*a,**kw)
            if _is_task(self):
                result.info['lift_off']=dict(self.task.module.SCORER.last)
            return result
        cls.reset,cls._task_step=reset,_task_step
        cls._lift_off_cabinet_bridge_v1=True
    if 'yam_arena.scene' in sys.modules:
        patch_scene(sys.modules['yam_arena.scene'])
    elif not any(getattr(f,'lift_off_scene_hook',False) for f in sys.meta_path):
        sys.meta_path.insert(0,LiftOffSceneFinder())
