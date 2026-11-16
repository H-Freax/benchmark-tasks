"""Move a full-size granite mortar and its loose pestle to the clear work zone.
Two free rigid bodies, ordinary contact, no task-controlled forces or pose writers.
"""
from pathlib import Path
import math,sys,json,os
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as mk
import robot_clearance
import yam_arena.demo as yd
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP,Box
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=1.
STONE=(.45,.43,.40);kit=mk.Kit();N=32
M0=np.array([.24,0.,TOP]);GOAL=np.array([.30,-.16,TOP])
R=.070;RI=.055;H=.080;FLOOR=.018
# Closed 32-sided bottom and individually convex annular sectors preserve the cavity.
def cylinder(r,z0,z1):
 v=[(r*math.cos(2*math.pi*i/N),r*math.sin(2*math.pi*i/N),z)for z in(z0,z1)for i in range(N)]
 f=[tuple(range(N-1,-1,-1)),tuple(range(N,2*N))]+[(i,(i+1)%N,(i+1)%N+N,i+N)for i in range(N)]
 return mk.convex(v,f)
wall=[cylinder(R,0,FLOOR)]
for i in range(N):
 a,b=2*math.pi*i/N,2*math.pi*(i+1)/N
 v=[(r*math.cos(t),r*math.sin(t),z)for z in(FLOOR,H)for r,t in((RI,a),(R,a),(R,b),(RI,b))]
 wall.append(mk.convex(v,[(3,2,1,0),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]))
# Two integral stone ears, realistic grip width 30 mm; no invisible grasp sites.
for y in(-.079,.079):wall.append(mk.box((.030,.036,.018),at=(0,y,.047)))
# Nominal granite density applied to actual union geometry (ears overlap ring).
# Mass estimate 1.85 kg; source/measurement must be checked before publication.
MASS_M=1.85;MASS_P=.48
kit.part('mortar',M0,wall,mass=MASS_M,color=STONE,contact_offset=.0005)
# Geometric floor/rim contact, approximately 53 degrees.
A=math.radians(52.95);PR=mk.axis_angle((0,1,0),A);AXIS=PR[:,2]
# Lower head end at inner floor, shaft leans on the opposite rim. COM remains
# between bottom and rim supports; gravity does not force an immediate spill.
LOW=np.array([-.025,0,FLOOR+.024*math.sin(A)])
PLOCAL=LOW+AXIS*.090
P0=M0+PLOCAL
pestle=[cylinder(.024,-.090,-.040),cylinder(.014,-.040,.090)]
kit.part('pestle',P0,pestle,mass=MASS_P,color=(.50,.48,.45),quat=mk.mat_to_quat(PR),contact_offset=.0005)
# Four 1 mm-thick painted-border equivalents lie outside the full goal footprint.
marks=[]
for i,(dx,dy,size)in enumerate(((0,-.12,(.21,.003,.001)),(0,.12,(.21,.003,.001)),(-.105,0,(.003,.24,.001)),(.105,0,(.003,.24,.001)))):
 marks.append(Box('target_mark_'+str(i),pos=tuple(GOAL+[dx,dy,.0005]),size=size,color=(.24,.48,.35)))
OBJECTS=[*kit.objects(),*marks]

def _metrics(objects):
 m,mq=kit.pose(objects,'mortar');p,pq=kit.pose(objects,'pestle');mr=mk.quat_to_mat(mq);pr=mk.quat_to_mat(pq)
 head=p+(np.asarray(pestle[0].vertices)+np.asarray(pestle[0].at))@pr.T
 local=(head-m)@mr
 normals=np.array([(math.cos((i+.5)*2*math.pi/N),math.sin((i+.5)*2*math.pi/N))for i in range(N)])
 tilt=math.degrees(math.acos(float(np.clip(mr[2,2],-1,1))))
 # Broad safe final states: any pestle orientation resting inside the cavity;
 # no requirement to keep a scripted pose, use a particular grip, or ordering.
 sub=local[local[:,2]<=H+.003];center=local.mean(0)
 contained=bool(len(sub)>0 and np.all(sub[:,:2]@normals.T<=RI*math.cos(math.pi/N)+.003) and local[:,2].min()>=FLOOR-.003 and center[2]<=H-.010 and np.all(center[:2]@normals.T<=RI*math.cos(math.pi/N)-.003))
 placed=bool(np.linalg.norm(m[:2]-GOAL[:2])<=.025 and abs(m[2]-TOP)<=.004 and tilt<=5.)
 stable=all(objects[n].resting(max_speed=.01,max_spin=.17)for n in('mortar','pestle'))
 clear=robot_clearance.check(objects,margin=.020)['clear']
 safe=not mk.MEMORY.get('off_table',False) and bool(objects['mortar'].low[2]>=TOP-.025 and objects['pestle'].low[2]>=TOP-.025)
 return dict(placed=placed,contained=contained,stable=stable,clear=clear,safe=safe,tilt_deg=tilt)

def score(objects):
 v=_metrics(objects)
 if not v['safe']:return 0.
 if not v['placed']:
  m,_=kit.pose(objects,'mortar')
  return .25 if np.linalg.norm(m[:2]-GOAL[:2])<=.09 and v['contained'] and v['stable'] else 0.
 if not v['contained']:return .50 if v['stable'] and v['clear'] else .25
 return 1. if v['stable'] and v['clear'] else .75

def success(objects):return score(objects)==1.
def _encoders():
 out={}
 for arm in('left_robot','right_robot'):
  robot=mk._EP['env'].unwrapped.scene[arm];v=getattr(robot.data.joint_pos,'torch',robot.data.joint_pos)
  if hasattr(v,'detach'):v=v.detach().cpu().numpy()
  out[arm]=dict(zip(robot.joint_names,np.asarray(v)[0].tolist()))
 return out
def observe(objects):
 if any(objects[n].low[2]<TOP-.05 for n in('mortar','pestle')):mk.MEMORY['off_table']=True
 if mk._EP.get('env') is not None and mk._EP.get('step',0)%20==0:
  print('MORTAR_OBS '+json.dumps({'step':mk._EP.get('step'),'mortar_origin':kit.pose(objects,'mortar')[0].tolist(),'pestle_origin':kit.pose(objects,'pestle')[0].tolist(),'mortar_quat_xyzw':kit.pose(objects,'mortar')[1].tolist(),'pestle_quat_xyzw':kit.pose(objects,'pestle')[1].tolist(),'encoders':_encoders(),'metrics':_metrics(objects),'score':score(objects)}),flush=True)
kit.on_step(observe)

# Downward grasp can pitch with the pestle while retaining ordinary IK/actions.
class Ang(float):
 def __new__(cls,yaw=0.,pitch=0.):x=super().__new__(cls,yaw);x.pitch=pitch;return x
stock_grip=getattr(yd.grip_rotation,'_mortar_stock',yd.grip_rotation)
def grip(yaw=0.):return mk.axis_angle((0,1,0),getattr(yaw,'pitch',0.))@stock_grip(float(yaw))
grip._mortar_stock=stock_grip;yd.grip_rotation=grip
class Seq:
 def __init__(self):self.w=[];self.t=0.
 def go(self,s,p,f,label,yaw=0.,pitch=0.):self.w.append(Waypoint(s,tuple(map(float,p)),f,label,Ang(yaw,pitch)));self.t+=s;return self
 def wait(self,s,label):w=self.w[-1];return self.go(s,w.pos,w.finger,label,float(w.yaw),w.yaw.pitch)
def demo():
 # Ordinary staging route. Each carry acts on a real independent body. The
 # unoccupied arm remains clear; all alternative successful strategies are legal.
 variant=os.environ.get('DEMO_VARIANT','')
 if variant not in('', 'mortar_only', 'keep_held'):raise ValueError('Unknown DEMO_VARIANT: '+variant)
 left,right=Seq(),Seq();tip=.0128425;delta=GOAL-M0
 park={'left':np.array([.14,.38,.90]),'right':np.array([.14,-.38,.90])}
 lg=M0+[0,.079,.047+tip]
 phi=math.acos(-.010/.130)-A;stageR=mk.axis_angle((0,1,0),A+phi)
 pv=np.concatenate([np.asarray(b.vertices)+b.at for b in pestle])
 stage=np.array([.20,-.37,TOP-(pv@stageR.T)[:,2].min()])
 shaft=np.array([0,0,.060]);basepitch=-math.pi/2
 def tcp(p,R,pitch):return p+R@shaft+mk.axis_angle((0,1,0),pitch)@np.array([0,0,tip])
 rg=tcp(P0,PR,basepitch);sg=tcp(stage,stageR,basepitch+phi);fg=tcp(GOAL+PLOCAL,PR,basepitch)
 def align():
  end=max(left.t,right.t)
  if left.t<end:left.wait(end-left.t,'remain clear while other arm works')
  if right.t<end:right.wait(end-right.t,'remain clear while other arm works')
 left.go(2,park['left'],-.035,'park left clear');right.go(2,park['right'],-.035,'park right clear');align()
 right.go(2,rg+[-.070,0,0],-.035,'approach exposed upper shaft from robot side',0,basepitch).go(2,rg,-.035,'around upper pestle shaft',0,basepitch).go(1,rg,0,'grasp loose pestle',0,basepitch).go(3,rg+[0,0,.090],0,'lift head fully above mortar rim',0,basepitch).go(2,rg+[0,-.150,.090],0,'withdraw clear of mortar at full height',0,basepitch)
 right.go(3,tcp(stage+[0,0,.040],stageR,basepitch+phi),0,'turn only after clearing mortar',0,basepitch+phi).go(3,sg,0,'set real pestle on tabletop',0,basepitch+phi).go(1,sg,-.035,'release staged pestle',0,basepitch+phi).go(2,sg+[0,0,.055],-.035,'clear staged pestle',0,basepitch+phi).go(2,park['right'],-.035,'park right clear');align()
 left.go(2,lg+[0,0,.070],-.035,'above mortar ear',-math.pi/2).go(2,lg,-.035,'around real stone ear',-math.pi/2).go(1,lg,0,'grasp mortar ear',-math.pi/2).go(2,lg+[0,0,.030],0,'lift full-size mortar',-math.pi/2).go(4,lg+delta+[0,0,.030],0,'carry mortar to work area',-math.pi/2).go(2,lg+delta,0,'set mortar on table',-math.pi/2).go(1,lg+delta,-.035,'release mortar',-math.pi/2).go(2,lg+delta+[0,0,.035],-.035,'withdraw from mortar',-math.pi/2).go(2,park['left'],-.035,'park left clear');align()
 if variant=='mortar_only':
  left.wait(3,'hands off mortar-only result');right.wait(3,'leave staged pestle on tabletop')
  return {'left':left.w,'right':right.w}
 right.go(2,sg+[0,0,.055],-.035,'above staged pestle',0,basepitch+phi).go(2,sg,-.035,'around real staged shaft',0,basepitch+phi).go(1,sg,0,'grasp staged pestle',0,basepitch+phi).go(2,sg+[0,0,.040],0,'lift clear of tabletop',0,basepitch+phi).go(3,fg+[0,-.150,.090],0,'turn clear of bowl',0,basepitch).go(2,fg+[0,0,.090],0,'carry above rim at full clearance',0,basepitch).go(2,fg,0,'lower head into cavity and onto floor',0,basepitch)
 if variant=='keep_held':
  right.wait(7,'hold real pestle inside bowl without releasing');align()
  return {'left':left.w,'right':right.w}
 right.go(1,fg,-.035,'release pestle',0,basepitch).go(2,fg+[-.070,0,0],-.035,'withdraw sideways from pestle',0,basepitch).go(2,park['right'],-.035,'park right clear').wait(3,'hands off final settling');align()
 return {'left':left.w,'right':right.w}
