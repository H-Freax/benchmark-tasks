"""Independent endpoint grade from measured outcome/state; no policy phase inputs."""
import math


def grade(row):
 try:
  if row.get('known')is not True:return 0.
  f=row['fluids'];keys=('water_ml','oil_ml','cup_water_ml','cup_oil_ml','spill_ml')
  if any(isinstance(f[k],bool)or not math.isfinite(f[k])or f[k]<-1e-7 for k in keys):return 0.
  if abs(sum(f[k]for k in keys)-180)>1e-4 or f.get('invalid_reason'):return 0.
  if not math.isfinite(row['flow_ml_s'])or not math.isfinite(row['stable_s']):return 0.
  if f['spill_ml']>1 or f['cup_oil_ml']>.5 or not row['closed']or row['flow_ml_s']>.005:return 0.
  # Milestones are recovered liquid fractions, never elapsed motion.
  recovered=f['cup_water_ml']
  if recovered<50:return 0.
  score=.25 if recovered<75 else .5 if recovered<97 else .75
  if recovered>=97 and f['oil_ml']>=79.5 and row['stopper_safe']and row['receiver_released']and row['robot_clear']and row['stable_s']>=1:return 1.
  return score
 except(KeyError,TypeError,ValueError,OverflowError):return 0.
