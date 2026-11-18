"""Pose-based held-valve settle diagnostic, separate from task scoring.

Native angular velocity is always retained; disagreement is never called zero.
Only fresh consecutive pose differences authorize this scripted release probe.
"""
import math

class PoseSettle:
 def __init__(self):self.previous=None;self.count=0
 def observe(self,row):
  result=dict(known=False,pose_settled=False,fresh_quiet_count=0,native_axis_angular_rad_s=None,pose_difference_angular_rad_s=None,native_velocity_disagrees=None,all_velocity_channels_quiet=False)
  try:
   if not row or row.get('known')is not True:raise ValueError('actual_observation_unknown')
   step=row['physics_step'];t=row['time_s'];angle=row['valve_angle_rad'];flow=row['flow_ml_s'];omega=row['velocities']['stopcock'][4]
   if type(step)is not int or not all(math.isfinite(v)for v in [t,angle,flow,omega]):raise ValueError('nonfinite_or_invalid_actual_valve')
   result.update(known=True,native_axis_angular_rad_s=omega)
   speed=None
   if self.previous is not None:
    before,bt,ba=self.previous
    if step-before==12 and t>bt:speed=(angle-ba)/(t-bt)
   self.previous=(step,t,angle)
   if speed is not None:
    quiet=abs(angle)<=math.radians(3)and flow<=.005 and abs(speed)<=.02
    self.count=self.count+1 if quiet else 0
    result.update(pose_difference_angular_rad_s=speed,native_velocity_disagrees=abs(omega-speed)>.1,all_velocity_channels_quiet=abs(omega)<=.02 and abs(speed)<=.02)
   else:self.count=0
   result.update(fresh_quiet_count=self.count,pose_settled=self.count>=10)
  except(KeyError,TypeError,ValueError,IndexError,OverflowError)as exc:
   self.previous=None;self.count=0;result['reason']=str(exc)
  return result
