"""CPU kinematic/collision checks and parent-only native smoke.

No native imports occur in --cpu-check or --dry-run. --native acquires the
exact shared GPU lock before importing the native implementation through reset.
"""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

ROOT=Path(__file__).resolve().parent
LOCK=Path(os.environ.get('ROBOCURVE_GPU_LOCK','/tmp/robocurve-gpu.lock'))
_GPU_LEASES=[]


def fingerprint():
    files={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
           for p in sorted(ROOT.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
           and '.pytest_cache' not in p.parts}
    return {'files':files,'sha256':hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()}


@contextmanager
def gpu_lease():
    if not LOCK.is_file():
        raise RuntimeError(f'parent shared GPU lock missing: {LOCK}')
    handle=LOCK.open('a+')
    try:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError('GPU lock busy; parent must queue this unchanged command') from exc
    _GPU_LEASES.append(handle)  # hold through Kit atexit, no early unlock
    yield handle


def _door_origin(g,label,target):
    if label in ('approach','insert','grasp','pull_without_lift','hold'):
        return g.START.copy()
    if label in ('release','pullback','retreat','settle','done'):
        return np.array([.460,0.,g.TABLE_Z]) if os.environ.get('LIFT_OFF_CABINET_DEMO')=='off_pad' else g.GOAL.copy()
    return np.array([target[0]+.038,0.,target[2]-.200])


def cpu_check(task):
    """Dense 20 Hz FK; 5 Hz actual URDF collision hull screen, no physics.

    Vertex signed distances are an inexpensive penetration kill test, not an
    exhaustive separating-axis swept collision certificate. Native grasps and
    pin contacts remain unverified, even if this necessary prerequisite passes.
    """
    from yam_arena.analysis import fk_link_poses_world,world_hulls,hulls_to_box,hull_set_distance
    from yam_arena.urdf import quat_xyzw_to_matrix
    g=task.module.g
    plans={side:ArmPlan(route,station.BASES[side]) for side,route in task.module.demo().items()}
    stats={side:dict(steps=0,max_position_error_m=0.,max_rotation_error_rad=0.,max_delta_rad=0.) for side in plans}
    previous={side:p.q.copy() for side,p in plans.items()}
    collision=dict(fixed_penetration_m=0.,panel_penetration_m=0.,inter_arm_penetration_m=0.,
                   samples=0,method='URDF convex hull vertex penetration, 5 Hz; necessary screen only')
    fixed=[(n,np.asarray(s),np.asarray(c)+g.START) for n,s,c in g.CABINET_BOXES]
    # Include the actual station camera stand/housing and tabletop plane.
    stand=station.camera_stand(station.top_camera())
    for name,part in stand.items():
        if part[0]=='box':
            _,size,pos,quat=part
            fixed.append((name,np.asarray(size),np.asarray(pos),quat_xyzw_to_matrix(np.asarray(quat))))
    for k in range(round(max(p.duration for p in plans.values())*20)+1):
        arm_hulls={}
        targets={}
        for side,plan in plans.items():
            action,label=plan.action7(k/20)
            target,rotation,_,_=plan.target(k/20)
            actual,rot=ik.tcp_pose(action[:6],station.BASES[side])
            err=float(np.linalg.norm(actual-target))
            angle=float(np.linalg.norm(ik._rot_error(rotation,rot)))
            delta=float(np.max(abs(action[:6]-previous[side])))
            previous[side]=action[:6]
            s=stats[side]
            s['steps']=k
            s['max_position_error_m']=max(s['max_position_error_m'],err)
            s['max_rotation_error_rad']=max(s['max_rotation_error_rad'],angle)
            s['max_delta_rad']=max(s['max_delta_rad'],delta)
            if err>.002 or angle>.02 or delta>.12 or np.any(action[:6]<ik.LOWER) or np.any(action[:6]>ik.UPPER):
                raise RuntimeError(f'CPU IK failed: {side} step {k} {label}: {err=}, {angle=}, {delta=}')
            if k%4==0:
                q={n:float(v) for n,v in zip(ARM_JOINTS,action[:6],strict=True)}
                q.update({n:float(action[6]*FINGER_OPEN) for n in FINGER_JOINTS})
                hulls=world_hulls(fk_link_poses_world(q,station.BASES[side]))
                arm_hulls[side]=hulls
                targets[side]=(label,target)
                for row in fixed:
                    name,size,center,*rotation_fixed=row
                    distance,link=hulls_to_box(hulls,center,rotation_fixed[0] if rotation_fixed else np.eye(3),size)
                    depth=max(0.,-float(distance))
                    if depth>collision['fixed_penetration_m']:
                        collision.update(fixed_penetration_m=depth,worst_fixed=dict(step=k,side=side,link=link,object=name))
                origin=_door_origin(g,label,target)
                distance,link=hulls_to_box(hulls,origin+[0,0,.2],np.eye(3),g.PANEL_SIZE)
                depth=max(0.,-float(distance))
                if depth>collision['panel_penetration_m']:
                    collision.update(panel_penetration_m=depth,worst_panel=dict(step=k,side=side,link=link))
                # Exclude stationary bases which intersect the support tabletop by design.
                moving_pts=np.vstack([h.vertices for n,h in hulls if n not in ('base','link1')])
                if moving_pts[:,2].min()<g.TABLE_Z-.0005:
                    raise RuntimeError(f'CPU arm/table penetration: {side} step {k}')
        if k%4==0:
            # Broadphase excludes every disjoint convex pair before expensive
            # face-plane queries (the upstream helper only excludes >5 cm gaps).
            left,right=arm_hulls['left'],arm_hulls['right']
            lo_l=np.stack([h.vertices.min(0) for _,h in left])
            hi_l=np.stack([h.vertices.max(0) for _,h in left])
            lo_r=np.stack([h.vertices.min(0) for _,h in right])
            hi_r=np.stack([h.vertices.max(0) for _,h in right])
            overlap=np.minimum(hi_l[:,None,:],hi_r[None,:,:])-np.maximum(lo_l[:,None,:],lo_r[None,:,:])
            pairs=np.argwhere(np.all(overlap>0,axis=2))
            for i,j in pairs:
                distance,pair=hull_set_distance([left[i]],[right[j]])
                depth=max(0.,-float(distance))
                if depth>collision['inter_arm_penetration_m']:
                    collision.update(inter_arm_penetration_m=depth,worst_arm_pair=dict(step=k,pair=pair))
            collision['samples']+=1
    if any(collision[n]>.0005 for n in ('fixed_penetration_m','panel_penetration_m','inter_arm_penetration_m')):
        raise RuntimeError('CPU collision kill: '+json.dumps(collision))
    return dict(native_run=False,score=None,media=None,ik=stats,collision=collision,door_mass_kg=g.DOOR_MASS,
                panel_size_m=g.PANEL_SIZE.tolist(),source=fingerprint())


def inspect_command(task,output,max_steps):
    return ['inspect-robots','run','--embodiment','yam_arena','--policy','yam_scripted',
            '-E',f'task={task.path}','-E','object_jitter=0','-E','control_hz=20',
            '-E','cam_width=640','-E','cam_height=360','--instruction',task.module.INSTRUCTION,
            '--scorer','task_score','--max-steps',str(max_steps),'--max-action-delta','.12',
            '--no-prompt','--no-rerun','--no-rerun-save','--store-frames','--log-dir',str(output)]


def _plain(value):
    if isinstance(value,np.ndarray):
        return value.tolist()
    if isinstance(value,dict):
        return {str(k):_plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):
        return [_plain(v) for v in value]
    if isinstance(value,np.generic):
        return value.item()
    return value


def native_smoke(task,output,variant,max_steps,resets):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from yam_arena.scripted import YamScriptedPolicy
    from PIL import Image
    with gpu_lease():
        source=fingerprint()
        emb=YamArenaEmbodiment(task=str(task.path),object_jitter=0,control_hz=20,cam_width=640,cam_height=360)
        policy=YamScriptedPolicy(hold_s=3.)
        scene=Scene(id='lift-off-cabinet-door-to-pad',instruction=task.module.INSTRUCTION)
        reset_rows=[]
        actions=[]
        traces=[]
        result_path=output/'native-result.json'
        receipt=dict(native_run=True,source=source,variant=variant,status='started',score=None,media=None)
        result_path.write_text(json.dumps(receipt,indent=2,allow_nan=False))
        try:
            for i in range(resets):
                obs=emb.reset(scene,seed=0)  # official bootstrap, physical reset, settle
                sample=emb.task.module.bridge.sample_native(emb)
                reset_rows.append(_plain(sample))
                frame=obs.images['top_cam'].copy()
                np.save(output/f'setup-reset-{i}.npy',frame)
                Image.fromarray(frame).save(output/f'setup-reset-{i}.png')
                if i<resets-1:
                    dirty=np.r_[[0.,1.04,1.,-.6,0.,0.],1.,[0.,1.04,1.,-.6,0.,0.],1.]
                    emb.step(Action(data=dirty))
            policy.bind(emb.info)
            policy.reset(scene)
            for k in range(max_steps):
                action=policy.act(obs).actions[0]
                if action.meta.get('request_stop'):
                    break
                result=emb.step(action)
                obs=result.observation
                actions.append(np.asarray(action.data,float).tolist())
                traces.append(_plain(dict(step=k+1,reading=emb.task.module.bridge.sample_native(emb),info=result.info)))
                if k%10==0:
                    np.save(output/f'top-{k:05}.npy',obs.images['top_cam'])
                    Image.fromarray(obs.images['top_cam']).save(output/f'top-{k:05}.png')
                if result.terminated:
                    break
            if source!=fingerprint():
                raise RuntimeError('task files changed during run')
            final=emb.task.module.SCORER.last
            receipt.update(status='completed',steps=len(actions),resets=reset_rows,score=final['score'],success=final['success'],final=final,
                           note='scripted native diagnostic only; no Dream/model run or uploaded media')
            return receipt
        except Exception as exc:
            receipt.update(status='error',error_type=type(exc).__name__,error=str(exc),steps=len(actions),resets=reset_rows)
            raise
        finally:
            (output/'actions.json').write_text(json.dumps(actions,allow_nan=False))
            (output/'trace.json').write_text(json.dumps(traces,allow_nan=False))
            result_path.write_text(json.dumps(_plain(receipt),indent=2,allow_nan=False))
            emb.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--cpu-check',action='store_true')
    mode.add_argument('--dry-run',action='store_true')
    mode.add_argument('--native',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--variant',choices=('positive','noop','pull_only','hover','held','off_pad'),default='positive')
    parser.add_argument('--max-steps',type=int,default=800)
    parser.add_argument('--resets',type=int,default=2)
    args=parser.parse_args()
    if not 1<=args.max_steps<=800 or args.resets<2:
        parser.error('max-steps must be 1..800; resets >=2')
    if args.native and args.output is None:
        parser.error('--native requires --output outside the task package')
    os.environ['LIFT_OFF_CABINET_DEMO']=args.variant
    task=load(ROOT/'task.py')
    if args.dry_run:
        receipt=dict(native_run=False,gpu_lock=str(LOCK),variant=args.variant,
                     command=inspect_command(task,args.output or Path('native-output'),args.max_steps))
    elif args.cpu_check:
        receipt=cpu_check(task)
    else:
        output=args.output.resolve()
        if output==ROOT or ROOT in output.parents:
            parser.error('native output must be outside package')
        output.mkdir(parents=True,exist_ok=False)
        receipt=native_smoke(task,output,args.variant,args.max_steps,args.resets)
    if args.cpu_check and args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'cpu-ready.json').write_text(json.dumps(receipt,indent=2,allow_nan=False))
    print(json.dumps(receipt,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
