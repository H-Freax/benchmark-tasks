"""Original declared full-size engineering-equivalent parts; m, kg, XYZW."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import station

TABLE=station.TABLE_TOP
CENTER=np.array([.36,0.,.925])
STOPPER=np.array([.36,0.,1.119])
CUP_START=np.array([.36,0.,TABLE])
CUP_TARGET=np.array([.36,0.,TABLE])
VALVE=np.array([.36,-.047,.925])
STOPPER_TRAY=np.array([.30,.08,TABLE+.004])
OUTLET=np.array([.36,0.,.852])
COLORS={'glass':(.65,.84,.91),'frame':(.35,.38,.40),'stopper':(.90,.90,.87),'valve':(.90,.90,.90),'mat':(.20,.24,.29),'cup':(.60,.82,.90)}


def quat(axis,angle):return tuple(Rotation.from_rotvec(np.array(axis)*angle).as_quat())

def ring(z,radius,thick,height,color,n=24):
 out=[]
 for i in range(n):
  a=2*math.pi*(i+.5)/n
  # Local radial x and tangential y, regular thin wall segment.
  out.append(((radius*math.cos(a),radius*math.sin(a),z),(thick,2*(radius+thick*.5)*math.tan(math.pi/n),height),color,quat([0,0,1],a)))
 return out


def conewall(z0,h,rlo,rhi,thick,color,n=24,layers=6):
 out=[]
 slope=(rhi-rlo)/h
 for j in range(layers):
  z=z0+h*(j+.5)/layers;r=rlo+(rhi-rlo)*(j+.5)/layers
  for i in range(n):
   a=2*math.pi*(i+.5)/n
   R=Rotation.from_euler('z',a).as_matrix()@Rotation.from_euler('y',math.atan(slope)).as_matrix()
   out.append(((r*math.cos(a),r*math.sin(a),z),(thick,2*(r+thick)*math.tan(math.pi/n),h/layers*math.sqrt(1+slope*slope)+.0004),color,tuple(Rotation.from_matrix(R).as_quat())))
 return out


def specs(pk):
 g=COLORS['glass'];f=COLORS['frame'];s=COLORS['stopper']
 # Cone bottom zero, top+90mm;50mm straight wall and40mm reduced neck.
 body=conewall(0,.09,.0035,.033,.002,g)+ring(.115,.033,.002,.05,g)+ring(.16,.0125,.003,.04,g)
 # Neck is connected by upper shoulder cone, omitted middle void is not solid.
 body+=conewall(.14,.01,.033,.0125,.002,g,layers=2)
 # Narrow downward outlet, bore physically open; valve block shell is annular.
 body+=ring(-.0475,.0035,.0015,.05,g,n=16)+ring(-.010,.006,.003,.020,g,n=16)
 # Ordinary stand: base and rear post, plus two visible arms clamping glass neck.
 stand=[((.12,.04,TABLE-CENTER[2]+.006),(.20,.16,.012),f),((.12,.04,.04),(.012,.012,.35),f),((.065,.04,.115),(.13,.012,.012),f)]
 stand+= [((.0,.040,.115),(.026,.020,.014),(.15,.15,.17))]
 # All frame geometry is expressed in the declared common world datum.
 funnel=pk.part('funnel_stand',CENTER,boxes=body+stand,mass=1.72,fixed=True,friction=.5,color=g,cylinders=[((0,-.024,0),.009,.048,s,'Y')])
 stopper=pk.part('stopper',STOPPER,boxes=[((0,0,0),(.025,.025,.016),s)],cylinders=[((0,0,-.022),.01098,.028,s,'Z')],mass=.028,friction=.65,color=s,damping=.5)
 # Common PTFE key on native revolute pivot; front bar is a normal48mm key,
 # not an added manipulation handle. Grip across its14mm thickness in Z.
 valve=pk.part('stopcock',VALVE,boxes=[((0,-.014,0),(.050,.016,.014),s)],cylinders=[((0,.003,0),.006,.034,s,'Y')],mass=.014,friction=.65,color=s,damping=.05)
 #250mL beaker approx70mm OD/100mm tall, glass base and hollow walls.
 cupboxes=[((0,0,.002),(.064,.064,.004),g)]+ring(.052,.032,.003,.10,g)
 # Ordinary commercial handle made of three glass/PP segments; leftside.
 cupboxes += [((0,.075,.040),(.014,.010,.050),s),((0,.054,.020),(.014,.044,.010),s),((0,.054,.060),(.014,.044,.010),s)]
 cup=pk.part('receiver',CUP_START,boxes=cupboxes,mass=.145,friction=.6,color=g,damping=.5)
 tray=pk.part('stopper_dish',STOPPER_TRAY,boxes=[((0,0,0),(.08,.07,.006),(.77,.79,.81))],mass=.05,fixed=True,friction=.6)
 return [funnel,stopper,valve,cup,tray]
