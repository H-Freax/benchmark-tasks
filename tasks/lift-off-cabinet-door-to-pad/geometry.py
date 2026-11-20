"""Self-authored real-size cut-list and contact-only slip-joint proxy (metres).

NH-40L/SS catalogue envelope: 40 x 30 x 7 mm, two knuckles. Internal pin
and running clearance are explicitly assumed, not manufacturer CAD. No joint,
release threshold, hidden grip fin or object-state write is used.
"""
from pathlib import Path
import numpy as np

TABLE_Z = .7475
PANEL_SIZE = np.array([.018,.350,.400])
WOOD_DENSITY = 450.
DOOR_MASS = float(np.prod(PANEL_SIZE)*WOOD_DENSITY + .021 + .100 + .012)
START = np.array([.360,0.,TABLE_Z+.010])  # door local origin = panel bottom centre
PAD_SIZE = np.array([.140,.410,.004])
PAD_CENTER = np.array([.250,0.,TABLE_Z+.002])
PAD_TOP = TABLE_Z+.004
GOAL = np.array([PAD_CENTER[0],0.,PAD_TOP])
HINGE_XY = np.array([.0145,.175])
HINGE_Z = (.050,.310)  # bottom of moving upper knuckle
PIN_RADIUS = .0015
PIN_LENGTH = .018
BORE_RADIUS = .0018
OUTER_RADIUS = .0035
SLEEVE_HEIGHT = .020
SHOULDER_HEIGHT = .020
SEGMENTS = 24
CONTACT_OFFSET = .00005  # two contact offsets leave 0.18 mm radial running clearance
WOOD = (.72,.55,.34)
STEEL = (.65,.67,.69)
DOOR_BOXES = (
    ('panel', tuple(PANEL_SIZE), (0.,0.,.200)),
    ('pull_bar', (.012,.202,.012), (-.038,0.,.200)),
    ('pull_post_left', (.023,.012,.012), (-.0205,.096,.200)),
    ('pull_post_right', (.023,.012,.012), (-.0205,-.096,.200)),
    *((f'moving_leaf_{i}',(.0015,.015,.040),(.00975,.1675,z)) for i,z in enumerate(HINGE_Z)),
    *((f'knuckle_web_{i}',(.003,.008,.020),(.011,.171,z+.010)) for i,z in enumerate(HINGE_Z)),
)
# Full empty cabinet: opening in the x-z plane, front y=.1795.
# Closing the open leaf +90 degrees would span x=.0145..3645, with 2.5 mm reveals.
CABINET_BOXES = (
    ('near_side',(.018,.240,.420),(.003,.2995,.200)),
    ('far_side',(.018,.240,.420),(.376,.2995,.200)),
    ('floor',(.355,.240,.018),(.1895,.2995,-.001)),
    ('roof',(.355,.240,.018),(.1895,.2995,.401)),
    ('back',(.355,.018,.384),(.1895,.4105,.200)),
    *((f'fixed_leaf_{i}',(.015,.0015,.040),(.0085,.18025,z)) for i,z in enumerate(HINGE_Z)),
    *((f'shoulder_web_{i}',(.006,.003,.018),(.0145,.178,z-.010)) for i,z in enumerate(HINGE_Z)),
)


def box_vertices(size, center):
    return np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size+center


def ring_segments():
    """24 convex annular sectors: the bore stays open under convex cooking."""
    out=[]
    for i in range(SEGMENTS):
        a,b=2*np.pi*np.array([i,i+1])/SEGMENTS
        xy=np.array([[r*np.cos(t),r*np.sin(t)] for r in (BORE_RADIUS,OUTER_RADIUS) for t in (a,b)])
        out.append(np.array([[x,y,z] for z in (0.,SLEEVE_HEIGHT) for x,y in xy]))
    return out


DOOR_VERTICES = np.vstack([
    *(box_vertices(s,c) for _,s,c in DOOR_BOXES),
    *(v+[*HINGE_XY,z] for z in HINGE_Z for v in ring_segments()),
])
BOTTOM_CORNERS = box_vertices(PANEL_SIZE,(0.,0.,.200))[[0,2,4,6]]
DOOR_LOW, DOOR_HIGH = DOOR_VERTICES.min(0),DOOR_VERTICES.max(0)
CABINET_VERTICES = np.vstack([*(box_vertices(s,c) for _,s,c in CABINET_BOXES),
    *(box_vertices((2*OUTER_RADIUS,2*OUTER_RADIUS,.040),(*HINGE_XY,z)) for z in HINGE_Z)])
CABINET_LOW,CABINET_HIGH = CABINET_VERTICES.min(0),CABINET_VERTICES.max(0)


def hinge_clearances(origin,rotation):
    """Conservative pin/knuckle occupancy, all fixed pins against both sleeves.

    In the moving body frame, enclose the solid pin by its axial segment plus
    radius; a candidate radial/axial overlap remains engaged (including the bore).
    This credits withdrawal only after both real pins have left both knuckles.
    """
    origin,rotation=np.asarray(origin,float),np.asarray(rotation,float)
    engaged=[]
    for pin_z in HINGE_Z:
        endpoints=np.array([[*HINGE_XY,pin_z],[*HINGE_XY,pin_z+PIN_LENGTH]])+START
        local=(endpoints-origin)@rotation
        a,b=local[:,:2]-HINGE_XY
        d=b-a
        u=float(np.clip(-np.dot(a,d)/max(np.dot(d,d),1e-30),0,1))
        radial=float(np.linalg.norm(a+u*d))
        for sleeve_z in HINGE_Z:
            axial=(local[:,2].max()+PIN_RADIUS >= sleeve_z-.002
                   and local[:,2].min()-PIN_RADIUS <= sleeve_z+SLEEVE_HEIGHT+.002)
            engaged.append(bool(axial and radial <= OUTER_RADIUS+PIN_RADIUS+.002))
    return {'detached':not any(engaged),'pin_sleeve_occupancy':engaged}


def _vec(values):
    return '('+', '.join(f'{float(v):.10g}' for v in values)+')'


def _collider_attrs(color):
    return f'''        bool physics:collisionEnabled = true
        float physxCollision:contactOffset = {CONTACT_OFFSET}
        float physxCollision:restOffset = 0
        color3f[] primvars:displayColor = [{_vec(color)}]
        rel material:binding:physics = </Body/Material>
'''


_APIS='["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]'

def _cube(name,size,center,color):
    return f'''    def Cube "{name}" (prepend apiSchemas = {_APIS})
    {{
        double size = 1
        double3 xformOp:translate = {_vec(center)}
        double3 xformOp:scale = {_vec(size)}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
{_collider_attrs(color)}    }}
'''


def _cylinder(name,radius,height,center):
    return f'''    def Cylinder "{name}" (prepend apiSchemas = {_APIS})
    {{
        double radius = {radius}
        double height = {height}
        uniform token axis = "Z"
        double3 xformOp:translate = {_vec(center)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
{_collider_attrs(STEEL)}    }}
'''


def _ring(name,vertices):
    return f'''    def Mesh "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"])
    {{
        point3f[] points = [{', '.join(_vec(v) for v in vertices)}]
        int[] faceVertexCounts = [4,4,4,4,4,4]
        int[] faceVertexIndices = [0,1,3,2,4,6,7,5,0,4,5,1,2,3,7,6,0,2,6,4,1,5,7,3]
        uniform token subdivisionScheme = "none"
        uniform token physics:approximation = "convexHull"
{_collider_attrs(STEEL)}    }}
'''


def _asset(children,dynamic=False,kinematic=False,friction=.5):
    metadata=' (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"])' if dynamic or kinematic else ''
    attrs=''
    if dynamic:
        attrs=f'''    bool physics:rigidBodyEnabled = true
    float physics:mass = {DOOR_MASS:.10g}
    point3f physics:centerOfMass = (-0.003, 0, 0.2)
    int physxRigidBody:solverPositionIterationCount = 32
    int physxRigidBody:solverVelocityIterationCount = 8
    bool physxRigidBody:enableCCD = true
'''
    elif kinematic:
        attrs='    bool physics:rigidBodyEnabled = true\n    bool physics:kinematicEnabled = true\n'
    return f'''#usda 1.0
(
    defaultPrim = "Body"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Body"{metadata}
{{
{attrs}    def Material "Material" (prepend apiSchemas = ["PhysicsMaterialAPI"])
    {{
        float physics:staticFriction = {friction}
        float physics:dynamicFriction = {friction*.8}
        float physics:restitution = 0
    }}
{children}}}
'''


def asset_texts():
    door=''.join(_cube(n,s,c,WOOD if n=='panel' else STEEL) for n,s,c in DOOR_BOXES)
    door+=''.join(_ring(f'ring_{j}_{i}',v+[*HINGE_XY,z]) for j,z in enumerate(HINGE_Z) for i,v in enumerate(ring_segments()))
    cabinet=''.join(_cube(n,s,c,WOOD if n in ('near_side','far_side','floor','roof','back') else STEEL) for n,s,c in CABINET_BOXES)
    for i,z in enumerate(HINGE_Z):
        cabinet+=_cylinder(f'shoulder_{i}',OUTER_RADIUS,SHOULDER_HEIGHT,(*HINGE_XY,z-SHOULDER_HEIGHT/2))
        cabinet+=_cylinder(f'pin_{i}',PIN_RADIUS,PIN_LENGTH,(*HINGE_XY,z+PIN_LENGTH/2))
    pad=_cube('rubber_pad',PAD_SIZE,(0,0,0),(.12,.42,.19))
    return {'door.usda':_asset(door,True,friction=.5),
            'cabinet.usda':_asset(cabinet,friction=.2),
            'landing_pad.usda':_asset(pad,kinematic=True,friction=.8)}


if __name__=='__main__':
    out=Path(__file__).resolve().parent/'assets'
    out.mkdir(exist_ok=True)
    for name,text in asset_texts().items():
        (out/name).write_text(text)
    print(f'Wrote {len(asset_texts())} USDA files; door mass {DOOR_MASS:.6f} kg; panel {PANEL_SIZE.tolist()} m')
