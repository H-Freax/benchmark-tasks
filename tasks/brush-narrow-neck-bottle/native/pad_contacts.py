"""Read-only two task bodies against every stock robot rigid-body partner."""
import numpy as np
LINKS=('base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right')
def array(v):
 v=getattr(v,'torch',v)
 if hasattr(v,'detach'):return v.detach().cpu().numpy().copy()
 if hasattr(v,'numpy'):return v.numpy().copy()
 return np.asarray(v).copy()
def decode(raw,filters,capacity=4096):
 nf,points,normals,seps,counts,starts=[array(v)for v in raw]
 if counts.shape!=(2,18)or starts.shape!=(2,18):raise ValueError('Actual contact pair ranges must be2x18')
 if not np.isfinite(counts).all()or not np.isfinite(starts).all()or (counts<0).any()or(starts<0).any():raise ValueError('Invalid pair range')
 if not np.equal(counts,np.floor(counts)).all()or not np.equal(starts,np.floor(starts)).all():raise ValueError('Noninteger pair range')
 used=set();out={}
 for i,sensor in enumerate(['feeding_bottle','bottle_brush']):
  out[sensor]={}
  for j,partner in enumerate(filters):
   n,s=int(counts[i,j]),int(starts[i,j])
   if n and (s+n>=capacity or any(s+n>len(a) for a in [nf.reshape(-1),points.reshape(-1,3),normals.reshape(-1,3),seps.reshape(-1)])):raise ValueError('Native contact buffer saturated/truncated')
   inds=set(range(s,s+n))
   if used&inds:raise ValueError('Overlapping native pair buffer ranges')
   used|=inds
   force=nf.reshape(-1)[s:s+n];p=points.reshape(-1,3)[s:s+n];normal=normals.reshape(-1,3)[s:s+n];sep=seps.reshape(-1)[s:s+n]
   if not all(np.isfinite(x).all()for x in [force,p,normal,sep]):raise ValueError('Nonfinite active pad contact')
   out[sensor][partner]=dict(count=n,start=s,normal_abs_sum_N=float(np.abs(force).sum()),normal_raw_N=force.tolist(),world_points_m=p.tolist(),world_normals=normal.tolist(),separations_m=sep.tolist())
 return dict(pairs=out,counts=counts.tolist(),starts=starts.tolist(),capacity=capacity,complete18partners=True)
class PadObserver:
 def __init__(self):self.view=None;self.filters=None;self.binding=None
 def bind(self,base):
  from pxr import Usd,UsdPhysics,PhysxSchema
  from isaaclab_physx.physics.physx_manager import PhysxManager
  root=base.scene.env_prim_paths[0];paths=[str(p.GetPath())for p in base.scene.stage.Traverse(Usd.TraverseInstanceProxies())if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  filters=[]
  for arm in ['LeftArm','RightArm']:
   actual=[p for p in paths if p.startswith(root+'/'+arm+'/')];by={p.rsplit('/',1)[-1]:p for p in actual}
   if len(actual)!=9 or set(by)!=set(LINKS):raise RuntimeError('Incomplete actual18partner inventory')
   filters.extend(by[n]for n in LINKS)
  sensors=[root+'/'+n+'/body'for n in ['feeding_bottle','bottle_brush']]
  for p in sensors:
   if not base.scene.stage.GetPrimAtPath(p).HasAPI(PhysxSchema.PhysxContactReportAPI):raise RuntimeError('Missing contact reporter')
  view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters,filters],max_contact_data_count=4096)
  if view is None or list(view.sensor_paths)!=sensors or list(view.filter_paths)!=[filters,filters]:raise RuntimeError('Actual18partner binding mismatch')
  self.view=view;self.filters=filters;self.binding=dict(sensors=sensors,filters=filters,capacity=4096)
 def observe(self,base,trace=None):
  if self.view is None:self.bind(base)
  raw=tuple(array(v)for v in self.view.get_contact_data(dt=float(base.physics_dt)))
  answer=decode(raw,self.filters);answer['physics_tick']=int(base._sim_step_counter);answer['physics_dt_s']=float(base.physics_dt)
  tangent=tuple(array(v)for v in self.view.get_friction_data(dt=float(base.physics_dt)))
  if len(tangent)!=4 or tangent[2].shape!=(2,18)or tangent[3].shape!=(2,18):raise RuntimeError('Tangential pair buffer unknown')
  if not all(np.isfinite(v).all()and(v>=0).all()and np.equal(v,np.floor(v)).all()for v in tangent[2:]):raise RuntimeError('Tangential pair ranges invalid')
  used=set()
  for n,s in zip(tangent[2].reshape(-1),tangent[3].reshape(-1)):
   n,s=int(n),int(s);inds=set(range(s,s+n))
   if used&inds:raise RuntimeError('Overlapping tangential pair ranges')
   if n and (s+n>=4096 or s+n>len(tangent[0])or s+n>len(tangent[1])):raise RuntimeError('Tangential buffer truncated')
   used|=inds
  answer['tangent_counts']=tangent[2].tolist();answer['tangent_starts']=tangent[3].tolist()
  extent=int((tangent[2]+tangent[3]).max())
  if extent>=4096:raise RuntimeError('Tangential buffer truncated')
  if not np.isfinite(tangent[0][:extent]).all()or not np.isfinite(tangent[1][:extent]).all():raise RuntimeError('Nonfinite actual tangential data')
  answer['tangent_world_vectors_raw_N']=tangent[0][:extent].tolist();answer['tangent_points_world_m']=tangent[1][:extent].tolist()
  return answer
def two_pad(answer,sensor,side):
 filters=answer['pairs'][sensor]
 wanted='/LeftArm/'if side=='left'else'/RightArm/'
 selected=[v for k,v in filters.items()if wanted in k and k.rsplit('/',1)[-1]in ['tip_left','tip_right']]
 return len(selected)==2 and all(v['count']>0 and v['normal_abs_sum_N']>1e-6 for v in selected)
