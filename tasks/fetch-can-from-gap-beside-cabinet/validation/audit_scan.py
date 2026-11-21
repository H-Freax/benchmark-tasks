"""Audit CSV completeness and selected limiting pairs independently with convex QP."""
from collections import Counter
import csv
import json
import math
from pathlib import Path
import sys

import fcl
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull

import scan_gap as scan


def read_rows(path):
    with path.open() as f:
        rows=list(csv.DictReader(f))
    for r in rows:
        for k,v in r.items():
            if v in ('True','False'):
                r[k]=v=='True'
            else:
                try:r[k]=float(v)
                except ValueError:pass
    return rows


def audit_pair(row):
    x,y=row['x_m'],row['y_m']
    traj=scan.trajectory(x,y,row['H_mm']/1000,row['h_mm']/1000,math.radians(row['yaw_deg']))
    active='left' if y>0 else 'right'
    if row['limiting_arm']==active:
        frame=min(traj['frames'],key=lambda f:abs(f[0]-row['limiting_time_s']))
        poses=frame[3]
    else:
        poses=scan.forward_kinematics(scan.YAM_URDF,scan.joint_map(scan.READY,scan.FINGER_OPEN))
    angle=math.radians(row['long_axis_deg'])
    ca,sa=math.cos(angle),math.sin(angle)
    wall_rot=np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]])
    origin=np.array([x,y,scan.TABLE_Z])
    base=np.asarray(scan.BASES[row['limiting_arm']])
    hull=next(h for h in scan.G if h.link==row['limiting_link'] and h.index==int(row['limiting_hull']))
    m=poses[hull.link]
    r=wall_rot.T@m[:3,:3]
    p=wall_rot.T@(m[:3,3]+base-origin)
    points=hull.vertices@r.T+p
    center=np.array([0,(-1 if row['limiting_wall']==0 else 1)*(row['w_mm']/2000+scan.WALL_THICKNESS/2),row['H_mm']/2000])
    half=np.array([scan.WALL_LENGTH/2,scan.WALL_THICKNESS/2,row['H_mm']/2000])
    hull.obj.setTransform(fcl.Transform(r,p))
    box=fcl.CollisionObject(fcl.Box(*(half*2)),fcl.Transform(center))
    gjk=max(0.,fcl.distance(hull.obj,box,fcl.DistanceRequest(),fcl.DistanceResult()))
    ch=ConvexHull(points)
    a,b=ch.equations[:,:3],ch.equations[:,3]
    start=points.mean(0)
    def objective(z):
        d=z[:3]-z[3:]
        return .5*float(d@d)
    def gradient(z):
        d=z[:3]-z[3:]
        return np.r_[d,-d]
    constraints=dict(type='ineq',fun=lambda z:-(a@z[:3]+b),
                     jac=lambda z:np.c_[-a,np.zeros_like(a)])
    result=minimize(objective,np.r_[start,np.clip(start,center-half,center+half)],
                    jac=gradient,constraints=constraints,
                    bounds=[(None,None)]*3+list(zip(center-half,center+half,strict=True)),
                    method='SLSQP',options=dict(ftol=1e-15,maxiter=1000))
    qp=math.sqrt(max(0.,2*result.fun))
    max_constraint_violation=max(float((a@result.x[:3]+b).max()),0.)
    return dict(config={k:row[k] for k in ('x_m','y_m','theta_deg','delta_mm','h_mm','c_mm')},
                limiting_link=row['limiting_link'],limiting_time_s=row['limiting_time_s'],
                csv_distance_mm=row['min_wall_distance_mm'],replayed_gjk_mm=gjk*1000,
                independent_qp_mm=qp*1000,qp_success=bool(result.success),qp_message=result.message,
                qp_constraint_violation_mm=max_constraint_violation*1000,
                gjk_vs_qp_error_mm=abs(gjk-qp)*1000,
                csv_vs_replay_error_mm=abs(gjk*1000-row['min_wall_distance_mm']),
                replay_ik_mm=traj['ik_mm'],replay_ik_deg=traj['ik_deg'])


def reference_sections():
    poses=scan.forward_kinematics(scan.YAM_URDF,scan.joint_map(scan.READY,scan.MEASURE['contact_finger_joint_m']))
    inv=np.linalg.inv(poses['gripper'])
    output={}
    for names,label in [(('gripper',),'palm'),(('link5',),'wrist_link5'),
                        (('tip_left','tip_right'),'finger_pair')]:
        edges=[];vertices=[]
        for hull in scan.G:
            if hull.link not in names:continue
            m=inv@poses[hull.link]
            v=hull.vertices@m[:3,:3].T+m[:3,3]-scan.TCP_OFFSET
            faces=ConvexHull(v).simplices
            ids=np.unique(np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1),axis=0)
            edges.append(v[ids]);vertices.append(v)
        e=np.concatenate(edges);v=np.concatenate(vertices)
        a=e[:,0];d=e[:,1]-a
        valid=np.abs(d[:,2])>1e-12
        a=a[valid];d=d[valid]
        best=np.zeros(2);at=np.zeros(2)
        for z in np.unique(v[:,2]):
            t=(z-a[:,2])/d[:,2]
            active=(t>=0)&(t<=1)
            p=a[active]+t[active,None]*d[active]
            p=np.concatenate([p,v[np.abs(v[:,2]-z)<1e-12]])
            width=np.ptp(p[:,:2],axis=0)
            at=np.where(width>best,z,at)
            best=np.maximum(best,width)
        output[label]=dict(max_closing_section_mm=float(best[0]*1000),
                           max_transverse_section_mm=float(best[1]*1000),
                           section_depth_from_TCP_mm=(at*1000).tolist())
    return output


def main():
    out=Path(sys.argv[1])
    rows=read_rows(out/'gap_scan.csv')
    variants=read_rows(out/'yaw_variants.csv')
    dims=json.loads((out/'gripper_dims.json').read_text())
    report=json.loads((out/'scan_summary.json').read_text())
    keys=('x_m','y_m','theta_deg','delta_mm','h_mm','c_mm')
    expected=set(scan.itertools.product((.16,.20,.24),(.10,.16,.22,-.10,-.16,-.22),
                                       (0,30,60,90),(5,15,25,40),(35,50,65),(4,6,8,10)))
    actual={tuple(r[k] for k in keys) for r in rows}
    grouped={}
    for r in variants:
        grouped.setdefault(tuple(r[k] for k in keys),[]).append(r)
    assertions=dict(grid_complete=actual==expected,unique_rows=len(actual)==len(rows)==3456,
                    variants_complete=len(variants)==6912,
                    every_frame_called=all(r['frames']==98 for r in variants),
                    every_waypoint_plus10=all(r['waypoint_plus10_checks']==5 for r in variants),
                    passing_thresholds_consistent=all(r['passed']==(r['ik_pass'] and r['plus10_pass'] and
                         r['wall_pass'] and r['joint_limits_pass'] and r['fingertip_floor_pass']) for r in rows),
                    selected_pass_iff_some_variant_passes=all(r['passed']==any(v['passed'] for v in
                         grouped[tuple(r[k] for k in keys)]) for r in rows),
                    script_hash_matches=report['script_sha256']==scan.sha(scan.__file__))
    sample=[]
    for delta in (5,15,25,40):
        r=report['best_by_delta'][str(delta)]
        if r:sample.append(r)
    for criterion in (lambda r:bool(r['contact_links']),lambda r:not r['ik_pass'],
                      lambda r:not r['plus10_pass'],lambda r:not r['wall_pass'] and not r['contact_links']):
        r=next((r for r in rows if criterion(r)),None)
        if r and not any(all(r[k]==s[k] for k in keys) for s in sample):sample.append(r)
    scan.init_worker(dims)
    dims['reference_maximum_cross_sections']=reference_sections()
    (out/'gripper_dims.json').write_text(json.dumps(dims,indent=2)+'\n')
    pair_checks=[audit_pair(r) for r in sample]
    assertions['independent_distance_checks']=all(p['qp_success'] and p['gjk_vs_qp_error_mm']<.005 and
             p['csv_vs_replay_error_mm']<.0001 and p['qp_constraint_violation_mm']<.001 for p in pair_checks)
    failed=[r for r in rows if not r['passed']]
    signatures=Counter(r['failure_reasons'] for r in failed)
    mismatch=sum((r['d_mm']>dims['palm_bottom_above_tcp_mm']) !=
                 (r['d_mm']>dims['L_palm_bottom_to_fingertip_mm']) for r in rows)
    result=dict(assertions=assertions,independent_distance_samples=pair_checks,
                failed_reason_combinations=dict(signatures),
                variant_failures_nonexclusive=dict(
                  total_rows=len(variants),passed=sum(r['passed'] for r in variants),
                  failed=sum(not r['passed'] for r in variants),
                  IK=sum(not r['ik_pass'] for r in variants),
                  plus10=sum(not r['plus10_pass'] for r in variants),
                  wall=sum(not r['wall_pass'] for r in variants),
                  joint_limit_violation=sum(not r['joint_limits_pass'] for r in variants),
                  joint_limit_saturation=sum(bool(r['limit_joints']) for r in variants)),
                nominal_palm_entry_classification_mismatch_rows=mismatch,
                total_action7_calls=sum(r['frames'] for r in variants)//4,
                total_plus10_solves=sum(r['waypoint_plus10_checks'] for r in variants)//4,
                note='Counts divide variant CSV rows by four because the four wall widths share a complete IK trajectory.',
                preliminary_runs=dict(pilot_combinations=8,pilot_passed=5,
                     analytic_convex_box_distance_cases=3,analytic_max_error_mm=8.7e-16),
                issues=['First full run stopped before scanning: sandbox denied writing into the attached worktree. Retried in /private/tmp; outputs copied afterward.',
                  'Initial dependency install was blocked by sandbox cache writes, and initial public asset download by sandbox DNS. Retried with reviewed network access and a temporary cache; both succeeded.',
                  'Migrated branch .venv uses Python 3.9.6 and lacks numpy/yam_arena. Scan used bundled Python 3.12.14 and temporary CPU dependencies.',
                  'Local Arena source pyproject version is 0.3.0, commit c7edf3a9d5d8276565208f72422f8f8a0806d6cf; the scan imports no Isaac or Arena runtime.',
                  'Can asset is the 6.2 public mirror; no Isaac Sim 6.1 / Arena 0.3.1 simulator validation was performed.',
                  'usd-core emitted ARCH_CACHE_LINE_SIZE != Arch_ObtainCacheLineSize() during measurement; USD mesh dimensions and SHA match the previously recorded asset.',
                  'd is measured from TCP, so palm entry starts at 70.501234 mm, not at L=83.343734 mm. Requested d/L classification retained; complete hull collision checks prevent omitted palm geometry.',
                  'At an exact downward pose, wrist link5 begins 134 mm above TCP, beyond the largest scan d=105.752791 mm. The requested d>L width rule therefore conservatively includes a wrist that is above the wall top.',
                  'The original task scaled the can to 66 mm; this scan uses 67 mm with height 100.752791 mm.',
                  'Joint-limit saturation is counted separately from violations because the unmodified framework solver clips every iterate.',
                  'All passes are CPU kinematics/geometry only: no contact simulation or proof of stable can extraction.',
                  'Failed local IK seeds/paths do not establish global infeasibility.'])
    (out/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    if not all(assertions.values()):raise SystemExit('Audit failed; inspect audit.json')


if __name__=='__main__':
    main()
