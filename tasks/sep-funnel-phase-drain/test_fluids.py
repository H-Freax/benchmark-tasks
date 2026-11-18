import unittest
import math
from fluid_model import Fluids
from grade import grade

class FluidTests(unittest.TestCase):
 def test_sealed_funnel_airlocks_before_useful_recovery(self):
  f=Fluids()
  for _ in range(4000): f.step(.05, angle=math.pi/2, vented=False, caught=True)
  self.assertLess(f.cup_water_ml,5)
  self.assertGreater(f.water_ml,95)
 def test_open_vent_transfers_dense_phase_then_oil(self):
  f=Fluids()
  for _ in range(2000):f.step(.05,angle=math.pi/2,vented=True,caught=True)
  self.assertAlmostEqual(f.cup_water_ml,100,places=4)
  self.assertGreater(f.cup_oil_ml,1)
  self.assertAlmostEqual(f.water_ml+f.oil_ml+f.cup_water_ml+f.cup_oil_ml+f.spill_ml,180,places=6)
 def test_missed_receiver_is_spill_not_recovery(self):
  f=Fluids()
  for _ in range(100):f.step(.05,angle=.8,vented=True,caught=False)
  self.assertGreater(f.spill_ml,1);self.assertEqual(f.cup_water_ml,0)
 def test_nominal_closed_valve_does_not_transfer(self):
  f=Fluids()
  for _ in range(300):f.step(.05,angle=0.,vented=True,caught=True)
  self.assertEqual(f.water_ml,100)
 def test_grade_safe_end_and_noop_and_bad_data(self):
  f=Fluids();f.water_ml=2;f.cup_water_ml=98
  end=dict(known=True,fluids=f.snapshot(),closed=True,stopper_safe=True,receiver_released=True,robot_clear=True,stable_s=1.1,flow_ml_s=0.)
  self.assertEqual(grade(end),1.)
  self.assertEqual(grade(dict(end,fluids=Fluids().snapshot())),0.)
  self.assertEqual(grade(dict(end,known=False)),0.)
  self.assertEqual(grade(dict(end,flow_ml_s=float('nan'))),0.)
 def test_no_credit_while_flowing_or_oil_contaminated(self):
  f=Fluids();f.water_ml=1;f.cup_water_ml=99;end=dict(known=True,fluids=f.snapshot(),closed=False,stopper_safe=True,receiver_released=True,robot_clear=True,stable_s=1.1,flow_ml_s=1.)
  self.assertEqual(grade(end),0.)
  f.cup_oil_ml=2;f.oil_ml=78;end.update(closed=True,flow_ml_s=0.,fluids=f.snapshot());self.assertEqual(grade(end),0.)
