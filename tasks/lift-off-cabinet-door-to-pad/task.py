"""Ordinary small plywood storage cabinet; open door on two slip-joint hinges.

Remove the open cabinet door and stand its bottom edge on the green rubber
bench pad. Let go and move both hands clear. Metres, x forward, y left, z up.
The 350 x 400 x 18 mm plywood door starts 90 degrees open. This is a real-size
small storage cabinet, with no locking tab or angle-dependent release.
"""
import importlib.util
from pathlib import Path
import sys
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

ROOT = Path(__file__).resolve().parent


def _local_module(name,filename):
    if name not in sys.modules:
        spec=importlib.util.spec_from_file_location(name,ROOT/filename)
        assert spec is not None and spec.loader is not None
        module=importlib.util.module_from_spec(spec)
        sys.modules[name]=module
        spec.loader.exec_module(module)
    return sys.modules[name]


g=_local_module('_lift_off_cabinet_geometry_v1','geometry.py')
bridge=_local_module('_lift_off_cabinet_native_bridge_v1','bridge.py')
bridge.install()
LIFT_OFF_CABINET_TASK = True
SCENE = 'station'
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 0.  # the measured scorer owns the continuous one-second window
INSTRUCTION = 'Remove the open cabinet door and stand its bottom edge on the green rubber pad. Let go and move both hands clear.'
# Start 0.2 mm above the seated shoulder plane, then settle under gravity.
# Exact initial cylinder/sector coplanarity produces a spurious lateral contact
# in the native narrow phase; this does not change the hinge or release geometry.
DOOR_RESET_ORIGIN = g.START + np.array([0.,0.,.0002])
OBJECTS = [
    bridge.USDPart('cabinet',tuple(g.START+(g.CABINET_LOW+g.CABINET_HIGH)/2),
                   size=tuple(g.CABINET_HIGH-g.CABINET_LOW),usd_path=str(ROOT/'assets/cabinet.usda'),origin=tuple(g.START)),
    bridge.USDPart('door',tuple(DOOR_RESET_ORIGIN+(g.DOOR_LOW+g.DOOR_HIGH)/2),mass=g.DOOR_MASS,
                   size=tuple(g.DOOR_HIGH-g.DOOR_LOW),usd_path=str(ROOT/'assets/door.usda'),origin=tuple(DOOR_RESET_ORIGIN)),
    bridge.USDPart('landing_pad',tuple(g.PAD_CENTER),size=tuple(g.PAD_SIZE),color=(.12,.42,.19),
                   usd_path=str(ROOT/'assets/landing_pad.usda'),origin=tuple(g.PAD_CENTER),kinematic=True),
]


class Scorer:
    """Current physical progress, never a best-stage or software release latch.

    Only measured pose, velocities, robot clearance and filtered support forces
    are inputs. CPU synthetic traces exercise arithmetic, not physics feasibility.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.history=[]
        self.previous_time=None
        self.last=dict(score=0.,success=False,measurement_valid=False)

    def update(self,sample):
        try:
            t=float(sample['time'])
            p=np.asarray(sample['origin'],float)
            q=np.asarray(sample['quat'],float)
            v=np.asarray(sample['lin_vel'],float)
            w=np.asarray(sample['ang_vel'],float)
            clear=float(sample['robot_clearance'])
            pad_force=float(sample['pad_up_force'])
            table_force=float(sample['table_up_force'])
            valid=(sample['measurement_valid'] is True and p.shape==(3,) and q.shape==(4,)
                   and v.shape==w.shape==(3,) and np.isfinite(np.r_[t,p,q,v,w,clear,pad_force,table_force]).all()
                   and abs(np.linalg.norm(q)-1)<1e-5 and t>=0 and min(clear,pad_force,table_force)>=0)
            if not valid:
                raise ValueError('incomplete or invalid physical sample')
            if self.previous_time is not None and (t<=self.previous_time or t-self.previous_time>.075):
                self.history.clear()
            self.previous_time=t
            r=quat_xyzw_to_matrix(q)
            vertices=g.DOOR_VERTICES@r.T+p
            bottom=g.BOTTOM_CORNERS@r.T+p
            detach=g.hinge_clearances(p,r)
            on_table=(np.all(vertices[:,0]>=-.19) and np.all(vertices[:,0]<=.91)
                      and np.all(np.abs(vertices[:,1])<=.65) and vertices[:,2].min()>=g.TABLE_Z-.003)
            withdrawn=bool(detach['detached'] and np.linalg.norm((p-g.START)[:2])>=.08)
            pad_lo,pad_hi=g.PAD_CENTER-g.PAD_SIZE/2,g.PAD_CENTER+g.PAD_SIZE/2
            footprint=bool(np.all(vertices[:,:2]>=pad_lo[:2]) and np.all(vertices[:,:2]<=pad_hi[:2]))
            upright=bool(r[2,2]>=np.cos(np.radians(5)))
            bottom_gaps=bottom[:,2]-g.PAD_TOP
            pad_support=bool(np.all(bottom_gaps>=-.001) and np.all(bottom_gaps<=.003)
                             and pad_force>=.85*g.DOOR_MASS*9.81)
            released=bool(clear>=.03)
            still=bool(np.linalg.norm(v)<.01 and np.linalg.norm(w)<.05)
            low=float(vertices[:,2].min())
            table_xy=bool(np.all(vertices[:,0]>=-.19) and np.all(vertices[:,0]<=.91)
                          and np.all(np.abs(vertices[:,1])<=.65))
            # Static tabletop: released, stationary physical surface contact is
            # verified by a <=1 mm full-body lowest-point gap; exclude pad overlap.
            pad_overlap=bool(np.all(vertices[:,:2].max(0)>=pad_lo[:2])
                             and np.all(vertices[:,:2].min(0)<=pad_hi[:2]))
            table_support=bool(table_xy and not pad_overlap and abs(low-g.TABLE_Z)<=.001)
            safe_support=bool(table_support or (abs(low-g.PAD_TOP)<=.003 and pad_force>=.85*g.DOOR_MASS*9.81))
            safe=bool(on_table and withdrawn and safe_support and released and still)
            full=bool(safe and footprint and upright and pad_support)
            if not safe:
                self.history.clear()
            else:
                self.history.append((t,vertices.copy(),full))
                self.history=[row for row in self.history if t-row[0]<=1.000001]
            window=bool(len(self.history)>=21 and t-self.history[0][0]>=1.-1e-6)
            displacement=float(np.linalg.norm(np.stack([pts for _,pts,_ in self.history])-self.history[0][1],axis=2).max()) if self.history else 0.
            stable=bool(window and displacement<=.002)
            success=bool(stable and full and all(f for _,_,f in self.history))
            progress=0.
            if on_table and detach['detached']:
                progress=.5 if withdrawn else .25
                if stable:
                    progress=1. if success else .75
            self.last=dict(score=progress,success=success,measurement_valid=True,time=t,
                           detached=bool(detach['detached']),withdrawn=withdrawn,on_table=bool(on_table),
                           footprint=footprint,upright=upright,pad_supported=pad_support,safe_support=safe_support,
                           released=released,still=still,pad_up_force_n=pad_force,table_up_force_n=table_force,
                           clearance_m=clear,bottom_gaps_m=bottom_gaps.tolist(),
                           window_s=t-self.history[0][0] if self.history else 0.,window_samples=len(self.history),
                           max_point_displacement_m=displacement,pin_sleeve_occupancy=detach['pin_sleeve_occupancy'])
            return self.last
        except (KeyError,TypeError,ValueError,IndexError,OverflowError):
            self.history.clear()
            self.previous_time=None
            self.last=dict(score=0.,success=False,measurement_valid=False)
            return self.last


def new_scorer():
    return Scorer()


SCORER=new_scorer()


def success(objects):
    return bool(SCORER.last['success'])


def score(objects):
    return float(SCORER.last['score'])


DEMO_VARIANTS=('positive','noop','pull_only','hover','held','off_pad')


def demo(variant=None):
    """Bimanual grasp of the ordinary horizontal pull, horizontal wrists.

    Only ordinary joint actions pass through ArmPlan; no attachment or pose write.
    A task-local waypoint rotation extension supplies reachable horizontal grips.
    """
    import os
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED
    variant=variant or os.environ.get('LIFT_OFF_CABINET_DEMO','positive')
    if variant not in DEMO_VARIANTS:
        raise ValueError(f'unknown demo variant {variant!r}')
    plans={}
    for side,sign in (('left',1),('right',-1)):
        if variant=='noop':
            plans[side]=[]
            continue
        # gripper +z approaches +x; closing axis +/-z; bar runs along y.
        rotation=np.column_stack(([0.,0.,-sign],[0.,sign,0.],[1.,0.,0.]))
        quat=tuple(ik.matrix_to_quat_xyzw(rotation))
        x=float(g.START[0]-.038)
        y=sign*.055
        z=float(g.START[2]+.200)
        goal_x=float((.460 if variant=='off_pad' else g.GOAL[0])-.038)
        goal_z=float((g.TABLE_Z if variant=='off_pad' else g.PAD_TOP)+.200)
        W=lambda duration,pos,finger,label: bridge.DoorWaypoint(duration,tuple(pos),finger,label,quat)
        seq=[W(4.,(x-.070,y,z),FINGER_OPEN,'approach'),
             W(3.,(x,y,z),FINGER_OPEN,'insert'),
             W(2.,(x,y,z),FINGER_CLOSED,'grasp')]
        if variant=='pull_only':
            seq+=[W(4.,(x-.040,y,z),FINGER_CLOSED,'pull_without_lift'),
                  W(2.,(x-.040,y,z),FINGER_CLOSED,'hold')]
        else:
            seq+=[W(3.,(x,y,z+.027),FINGER_CLOSED,'lift_clear'),
                  W(4.,(goal_x,y,z+.027),FINGER_CLOSED,'withdraw')]
            if variant=='hover':
                seq+=[W(3.,(goal_x,y,z+.027),FINGER_CLOSED,'hover')]
            else:
                seq+=[W(3.,(goal_x,y,goal_z),FINGER_CLOSED,'lower')]
                if variant=='held':
                    seq+=[W(3.,(goal_x,y,goal_z),FINGER_CLOSED,'held')]
                else:
                    retreat_x=.300 if variant=='off_pad' else goal_x
                    seq+=[W(2.,(goal_x,y,goal_z),FINGER_OPEN,'release')]
                    if variant=='off_pad':
                        seq+=[W(2.,(retreat_x,y,goal_z),FINGER_OPEN,'pullback')]
                    seq+=[W(3.,(retreat_x,sign*.240,goal_z),FINGER_OPEN,'retreat'),
                          W(2.,(retreat_x,sign*.240,goal_z),FINGER_OPEN,'settle')]
        plans[side]=seq
    # Measured loaded wrists lagged the original 27 mm lift; retain extraction
    # headroom, level the door during carry, and exit the pull before side travel.
    if variant not in ('noop','pull_only'):
        pitch=np.radians(-3.)
        co,si=np.cos(pitch),np.sin(pitch)
        ry=np.array([[co,0.,si],[0.,1.,0.],[-si,0.,co]])
        from yam_arena.urdf import quat_xyzw_to_matrix
        for side,seq in plans.items():
            sign=1 if side=='left' else -1
            base_rotation=np.column_stack(([0.,0.,-sign],[0.,sign,0.],[1.,0.,0.]))
            for w in seq:
                if w.label in ('lift_clear','withdraw','hover'):
                    w.pos=tuple(np.asarray(w.pos)+[0.,0.,.040])
                if w.label in ('withdraw','lower','release','pullback','retreat','settle'):
                    w.quat_xyzw=tuple(ik.matrix_to_quat_xyzw(ry@quat_xyzw_to_matrix(np.asarray(w.quat_xyzw))))
            if variant=='positive':
                for w in seq:
                    if w.label in ('retreat','settle'):
                        w.pos=(float(g.GOAL[0]-.078),sign*.055,float(g.PAD_TOP+.200))
                        w.quat_xyzw=tuple(ik.matrix_to_quat_xyzw(base_rotation))
                retreat=next(w for w in seq if w.label=='retreat')
                retreat.label='pullback'
                retreat.duration=2.
                index=seq.index(retreat)+1
                seq.insert(index,bridge.DoorWaypoint(3.,(float(g.GOAL[0]-.078),sign*.240,float(g.PAD_TOP+.200)),
                                                    retreat.finger,'retreat',retreat.quat_xyzw))
                next(w for w in seq if w.label=='settle').pos=(float(g.GOAL[0]-.078),sign*.240,float(g.PAD_TOP+.200))
    return plans
