"""Read-only complete630piece robot clearance for the mortar and pestle."""
import numpy as np
from yam_arena.analysis import link_hulls
import mechkit as mk

_BOUNDS=None
_SIGNS=np.array([(x,y,z)for x in(-1,1)for y in(-1,1)for z in(-1,1)],float)

def array(value):
    value=getattr(value,'torch',value)
    if hasattr(value,'detach'):value=value.detach().cpu().numpy()
    return np.asarray(value)

def bounds():
    global _BOUNDS
    if _BOUNDS is None:
        _BOUNDS={name:[((h.vertices.min(0)+h.vertices.max(0))/2,
            (h.vertices.max(0)-h.vertices.min(0))/2)for h in hs]
            for name,hs in link_hulls().items()}
        if sum(map(len,_BOUNDS.values()))!=315:
            raise ValueError('Expected315actualrobotcollisionpiecesperarm')
    return _BOUNDS

def check(objects,margin=.020):
    """ConservativeactualOBBs vs bothactualrigidAABBs; invalid data failsclosed."""
    validated_margin=None
    try:
        margin_array=np.asarray(margin,dtype=float)
        if margin_array.shape!=() or not np.isfinite(margin_array) or margin_array<0.:
            raise ValueError('Invalid clearance margin')
        margin=float(margin_array);validated_margin=margin
        base=mk._EP['env'].unwrapped
        origin=array(base.scene.env_origins)[0]
        if origin.shape!=(3,) or not np.isfinite(origin).all():
            raise ValueError('Invalid environment origin')
        pieces=[];required=bounds()
        for arm in('left_robot','right_robot'):
            robot=base.scene[arm];names=list(robot.body_names)
            if len(names)!=len(set(names))or not set(required)<=set(names):
                raise ValueError('Incompleteactualrobotcollisionlinks')
            pos=array(robot.data.body_pos_w)[0];quat=array(robot.data.body_quat_w)[0]
            if (pos.shape!=(len(names),3) or quat.shape!=(len(names),4)
                    or not np.isfinite(pos).all() or not np.isfinite(quat).all()
                    or np.any(np.abs(np.linalg.norm(quat,axis=1)-1.)>1e-3)):
                raise ValueError('Invalid actual robot collision poses')
            for name,hs in required.items():
                index=names.index(name);R=mk.quat_to_mat(quat[index])  # Installed IsaacLab 3 body_quat_w alias uses xyzw
                for number,(center,half)in enumerate(hs):
                    pieces.append((f'{arm}/{name}/{number}',R@center+pos[index]-origin,half,R))
        if len(pieces)!=630:raise ValueError('Incompleteactualrobotcollisionpieces')
        rigid=[]
        for name in('mortar','pestle'):
            low=np.asarray(objects[name].low,dtype=float);high=np.asarray(objects[name].high,dtype=float)
            if (low.shape!=(3,) or high.shape!=(3,)
                    or not np.isfinite(low).all() or not np.isfinite(high).all()
                    or np.any(low>high)):
                raise ValueError('Invalid rigid object bounds')
            rigid.append((low+high)/2+_SIGNS*(high-low)/2)
        centers=np.array([x[1]for x in pieces]);halves=np.array([x[2]for x in pieces])
        rotations=np.array([x[3]for x in pieces])
        if (not np.isfinite(centers).all() or not np.isfinite(halves).all()
                or not np.isfinite(rotations).all() or np.any(halves<0.)):
            raise ValueError('Invalid robot collision bounds')
        local=np.einsum('btvi,bij->btvj',np.array(rigid)[None]-centers[:,None,None,:],rotations)
        if not np.isfinite(local).all():
            raise ValueError('Invalid transformed clearance geometry')
        expanded=halves[:,None,:]+margin
        overlap=np.all((local.max(2)>=-expanded)&(local.min(2)<=expanded),axis=2)
        blocked=[(pieces[i][0],('mortar','pestle')[j])for i,j in np.argwhere(overlap)]
        return dict(valid=True,clear=not blocked,robot_piece_count=630,margin_m=margin,blocked_pairs=blocked,error=None)
    except (KeyError,AttributeError,ValueError,IndexError,TypeError,OverflowError)as error:
        return dict(valid=False,clear=False,robot_piece_count=0,margin_m=validated_margin,blocked_pairs=[],error=str(error))
