"""Read-only20Hz nativecontactdata. Missingpairsstayunknown."""
import numpy as np
class ContactReadback:
 def __init__(self,rt):self.rt=rt;self.reset()
 def reset(self):self.views={};self.errors={};self.installed=False
 def install(self,base):
  self.installed=True
  try:
   from isaaclab_physx.physics import PhysxManager
   parent=PhysxManager.get_physics_sim_view();root=base.scene.env_prim_paths[0]
  except Exception as e:self.errors['parent']=str(e);return
  pairs=[('centre_panel','left_panel'),('centre_panel','right_panel')]
  paths={a+'__'+b:(root+'/'+a+'/body',root+'/'+b+'/body')for a,b in pairs}
  try:
   from pxr import Usd,UsdPhysics
   for prim in base.scene.stage.Traverse(Usd.TraverseInstanceProxies()):
    path=str(prim.GetPath())
    if prim.HasAPI(UsdPhysics.RigidBodyAPI) and (path.startswith(root+'/LeftArm/') or path.startswith(root+'/RightArm/')) and 'tip_' in path:
     side='left'if '/LeftArm/'in path else'right'
     for target in ['centre_panel','left_panel','right_panel']:paths[target+'__'+side+'__'+path.rsplit('/',1)[-1]]=(root+'/'+target+'/body',path)
  except Exception as e:self.errors['fingerpaths']=str(e)
  # Plane prim path is discovered from the actual stage. Missing floor
  # binding remains unknown, never interpreted as absence of support force.
  try:
   candidates=[]
   for prim in base.scene.stage.Traverse(Usd.TraverseInstanceProxies()):
    if prim.HasAPI(UsdPhysics.CollisionAPI)and ('ground'in str(prim.GetPath()).lower()or'floor'in str(prim.GetPath()).lower()):candidates.append(str(prim.GetPath()))
   for target in ['centre_panel','left_panel','right_panel']:
    for index,path in enumerate(candidates):paths[target+'__floor_'+str(index)]=(root+'/'+target+'/body',path)
   if not candidates:self.errors['floor']='No actual floor collider path discovered'
  except Exception as e:self.errors['floor']=str(e)
  for key,(sensor,partner)in paths.items():
   try:
    v=parent.create_rigid_contact_view([sensor],filter_patterns=[[partner]],max_contact_data_count=128);v.get_contact_data(dt=float(base.physics_dt));self.views[key]=v
   except Exception as e:self.errors[key]=type(e).__name__+':'+str(e)
 def observe(self,base):
  if not self.installed:self.install(base)
  out={}
  for key,v in self.views.items():
   try:
    normal,point,direction,separation,count,start=[self.rt.arr(x)for x in v.get_contact_data(dt=float(base.physics_dt))];entries=[]
    for ix in np.ndindex(count.shape):
     for j in range(int(start[ix]),int(start[ix])+int(count[ix])):entries.append(dict(normal_force_N=float(np.asarray(normal[j]).reshape(-1)[0]),point_world_m=point[j].tolist(),normal_world=direction[j].tolist(),separation_m=np.asarray(separation[j]).tolist()))
    out[key]=dict(counts=count.tolist(),entries=entries)
   except Exception as e:out[key]=dict(unknown=type(e).__name__+':'+str(e))
  return dict(pairs=out,errors=self.errors,limitation='Lastsubsteponly20Hz; nocontactforcecalibrationclaim; missingisunknown')
