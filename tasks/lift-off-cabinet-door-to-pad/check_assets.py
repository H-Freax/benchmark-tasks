"""Standalone stock OpenUSD 26.11 gate. Never run inside the Isaac environment.

uv run --python 3.12 --with usd-core==26.11 --no-project python -B check_assets.py
"""
from pathlib import Path
import json
from pxr import Gf,Usd,UsdGeom,UsdPhysics

ROOT=Path(__file__).resolve().parent


def main():
    expected={'door.usda':((-0.044,-.175,0.),(.018,.1785,.4)),
              'cabinet.usda':((-.006,.1715,-.01),(.385,.4195,.41)),
              'landing_pad.usda':((-.07,-.205,-.002),(.07,.205,.002))}
    results=[]
    for name,(lo,hi) in expected.items():
        stage=Usd.Stage.Open(str(ROOT/'assets'/name))
        assert stage and stage.GetDefaultPrim().GetPath().pathString=='/Body',name
        assert UsdGeom.GetStageMetersPerUnit(stage)==1. and UsdGeom.GetStageUpAxis(stage)=='Z'
        assert not stage.GetRootLayer().subLayerPaths
        colliders=[]
        for prim in stage.Traverse():
            assert not prim.IsA(UsdPhysics.Joint),prim.GetPath()
            assert not prim.HasAuthoredReferences() and not prim.HasPayload(),prim.GetPath()
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                colliders.append(prim)
                assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                if prim.IsA(UsdGeom.Mesh):
                    assert prim.HasAPI(UsdPhysics.MeshCollisionAPI)
                    assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()=='convexHull'
        assert colliders,name
        body=stage.GetDefaultPrim()
        if name=='door.usda':
            assert body.HasAPI(UsdPhysics.RigidBodyAPI)
            assert abs(UsdPhysics.MassAPI(body).GetMassAttr().Get()-1.267)<1e-6
            assert len([p for p in colliders if p.GetName().startswith('ring_')])==48
        elif name=='landing_pad.usda':
            assert UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get()
        else:
            assert not body.HasAPI(UsdPhysics.RigidBodyAPI)
            assert stage.GetPrimAtPath('/Body/pin_0').IsA(UsdGeom.Cylinder)
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_])
        bounds=cache.ComputeWorldBound(body).ComputeAlignedRange()
        actual=(tuple(bounds.GetMin()),tuple(bounds.GetMax()))
        assert all(abs(a-b)<1e-6 for group,want in zip(actual,(lo,hi),strict=True) for a,b in zip(group,want,strict=True)),(name,actual,(lo,hi))
        results.append({'asset':name,'colliders':len(colliders),'low':actual[0],'high':actual[1]})
        print(name+': parsed')
    print(json.dumps({'stock_openusd':Usd.GetVersion(),'assets':results}))


if __name__=='__main__':
    main()
