"""Collider-free phase volumes in their actual vessel frames, driven by mL state."""
import math
import numpy as np
from fluid_model import height_ml,CONE_H,RADIUS


def layer_mesh(z0,z1,radius_at,n=48):
    """Closed piecewise-frustum mesh with a vertex ring at the cone shoulder."""
    levels=[z0]+([CONE_H] if z0<CONE_H<z1 else [])+[z1]
    points=[]
    for z in levels:
        radius=max(.00001,float(radius_at(z)))
        points.extend((radius*math.cos(i*2*math.pi/n),radius*math.sin(i*2*math.pi/n),z)for i in range(n))
    counts=[n];indices=list(reversed(range(n)))
    for band in range(len(levels)-1):
        for i in range(n):
            j=(i+1)%n
            indices.extend((band*n+i,band*n+j,(band+1)*n+j,(band+1)*n+i));counts.append(4)
    indices.extend(range((len(levels)-1)*n,len(levels)*n));counts.append(n)
    return points,counts,indices


def draw_phases(base,poses,fluids):
    from scene_spec import CENTER,OUTLET
    from pxr import UsdGeom,UsdShade,Gf,Sdf
    from scipy.spatial.transform import Rotation
    stage=base.scene.stage;root=base.scene.env_prim_paths[0]+'/phase_liquid_visual'

    def material(name,color):
        path=root+'/Looks/'+name
        mat=UsdShade.Material.Get(stage,path)
        if not mat:
            mat=UsdShade.Material.Define(stage,path)
            shader=UsdShade.Shader.Define(stage,path+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.45)
            shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
            shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(1.)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        return mat

    def mesh(name,z0,z1,radius_at,color,pose):
        obj=UsdGeom.Mesh.Define(stage,root+'/'+name)
        obj.CreateSubdivisionSchemeAttr('none');obj.CreateDoubleSidedAttr(True)
        obj.CreateVisibilityAttr('invisible' if z1-z0<1e-7 else 'inherited')
        points,counts,indices=layer_mesh(z0,max(z0+1e-7,z1),radius_at)
        obj.CreatePointsAttr([Gf.Vec3f(*p)for p in points]);obj.CreateFaceVertexCountsAttr(counts);obj.CreateFaceVertexIndicesAttr(indices)
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(material(name,color))
        ops=obj.GetOrderedXformOps()
        translation=next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate),None)or obj.AddTranslateOp()
        rotation=next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeOrient),None)or obj.AddOrientOp()
        translation.Set(Gf.Vec3d(*map(float,pose[:3])))
        q=pose[3:];rotation.Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
        # No CollisionAPI, RigidBodyAPI, joints, or object-state setters.

    hw=height_ml(fluids.water_ml);ht=height_ml(fluids.water_ml+fluids.oil_ml)
    radius=lambda z:min(RADIUS,RADIUS*z/CONE_H)*.975
    pose=[*CENTER,0.,0.,0.,1.]
    mesh('water',0.,hw,radius,(.025,.20,.85),pose)
    mesh('oil',hw,ht,radius,(.95,.58,.035),pose)
    # Same mL state and actual receiver transform; changing rigid mass remains omitted.
    cup=poses['receiver'][0];area=math.pi*.029**2
    cw=fluids.cup_water_ml/1e6/area;co=fluids.cup_oil_ml/1e6/area
    mesh('receiver_water',.004,.004+cw,lambda z:.029,(.025,.20,.85),cup)
    mesh('receiver_oil',.004+cw,.004+cw+co,lambda z:.029,(.95,.58,.035),cup)
    # Render the estimated stream only; this carries no hidden physical force.
    mouth=cup[:3]+Rotation.from_quat(cup[3:]).as_matrix()@np.array([0,0,.102])
    bottom=float(mouth[2]);top=float(OUTLET[2]);visible=fluids.flow_ml_s>.01 and top>bottom
    mesh('stream',0.,top-bottom if visible else 0.,lambda z:.0008,
         (.025,.20,.85)if fluids.water_ml>0 else(.95,.58,.035),[*OUTLET[:2],bottom,0.,0.,0.,1.])
