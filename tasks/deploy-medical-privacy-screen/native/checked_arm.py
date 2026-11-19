"""Task adaptation of MIT ordinaryArm: bounded endpoint IK and joint approach."""
import numpy as np
from scipy.optimize import least_squares
from ordinary_arm import Arm as DonorArm
from yam_arena import ik
class Arm(DonorArm):
 def solve_endpoint(self,p,R):
  def f(q):
   x,r=ik.tcp_pose(q,self.base);return np.r_[x-p,.3*ik._rot_error(R,r)]
  seeds=[self.q,(ik.LOWER+ik.UPPER)/2];rng=np.random.default_rng(2036)
  seeds.extend(rng.uniform(ik.LOWER,ik.UPPER)for _ in range(8));best=None
  for seed in seeds:
   fit=least_squares(f,np.clip(seed,ik.LOWER+1e-7,ik.UPPER-1e-7),bounds=(ik.LOWER,ik.UPPER),max_nfev=160,ftol=1e-10,xtol=1e-10,gtol=1e-10)
   if best is None or np.linalg.norm(fit.fun)<np.linalg.norm(best.fun):best=fit
   if np.linalg.norm(fit.fun[:3])<1e-5 and np.linalg.norm(fit.fun[3:])<1e-5:break
  if np.linalg.norm(best.fun[:3])>.002 or np.linalg.norm(best.fun[3:])/.3>np.radians(1):raise RuntimeError(('unreachable endpoint',self.name,best.fun.tolist()))
  return best.x
 def joint_approach(self,p,R,seconds,grip):
  q0=self.q.copy();q1=self.solve_endpoint(np.asarray(p),np.asarray(R));g0=self.grip
  for k in range(1,round(seconds*20)+1):
   u=ik.smoothstep(k/round(seconds*20));self.q=q0+u*(q1-q0);self.grip=g0+u*(grip-g0);self.pos,self.rot=ik.tcp_pose(self.q,self.base);self.cmd=np.r_[self.q,self.grip];yield
 def return_ready(self,seconds):
  from yam_arena.constants import READY
  q0=self.q.copy();q1=np.array(READY)
  for k in range(1,round(seconds*20)+1):
   u=ik.smoothstep(k/round(seconds*20));self.q=q0+u*(q1-q0);self.grip=1.;self.pos,self.rot=ik.tcp_pose(self.q,self.base);self.cmd=np.r_[self.q,1.];yield
