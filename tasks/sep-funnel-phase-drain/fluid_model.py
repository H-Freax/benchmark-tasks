"""Quasi-static immiscible water/edible-oil outflow; SI hydraulics, mL bookkeeping.

No script phase, robot command, stroke count or target volume enters dynamics.
Contact creates valve rotation and removes stopper in the native rigid scene.
"""
from dataclasses import dataclass,asdict
import math

P_ATM=101325.
G=9.81
RHO_WATER=998.
RHO_OIL=915.
# Declared original 250mL funnel equivalent: 90mm-high conical sump,
# top radius32mm; upper50mm cylinder. Orifice diameter2.5mm.
CONE_H=.09
RADIUS=.032
CONE_ML=math.pi*RADIUS**2*CONE_H/3*1e6
AREA=math.pi*RADIUS**2
GAS_EMPTY_ML=250.


def height_ml(volume):
 if volume<=0:return 0.
 if volume<=CONE_ML:return CONE_H*(volume/CONE_ML)**(1/3)
 return CONE_H+(volume-CONE_ML)/1e6/AREA


@dataclass
class Fluids:
 water_ml:float=100.
 oil_ml:float=80.
 cup_water_ml:float=0.
 cup_oil_ml:float=0.
 spill_ml:float=0.
 p_air_Pa:float=P_ATM
 sealed_pv:float|None=None
 flow_ml_s:float=0.
 invalid_reason:str|None=None

 def step(self,dt,*,angle,vented,caught,cup_tilt=0.):
  if not all(math.isfinite(x)for x in [dt,angle,cup_tilt])or not 0<dt<=.1:
   self.invalid_reason='Nonfinite_or_invalid_actual_input';return
  if not isinstance(vented,bool)or not isinstance(caught,bool):self.invalid_reason='Unknown_actual_seal_or_receiver';return
  if self.invalid_reason:return
  # Rigid cup no-slosh approximation holds only while nearly upright;
  # a large actual tilt irreversibly spills its retained contents.
  if cup_tilt>math.radians(35):
   self.spill_ml+=self.cup_water_ml+self.cup_oil_ml;self.cup_water_ml=self.cup_oil_ml=0.
  hwater=height_ml(self.water_ml);htotal=height_ml(self.water_ml+self.oil_ml)
  gas_ml=GAS_EMPTY_ML-self.water_ml-self.oil_ml
  if vented:self.p_air_Pa=P_ATM;self.sealed_pv=None
  else:
   if self.sealed_pv is None:self.sealed_pv=self.p_air_Pa*gas_ml
   self.p_air_Pa=self.sealed_pv/gas_ml
  density=RHO_WATER if self.water_ml>1e-9 else RHO_OIL
  head=G*(RHO_WATER*hwater+RHO_OIL*(htotal-hwater))
  delta=max(0.,head+self.p_air_Pa-P_ATM)
  # Bore-overlap area tends continuously to zero at closed transverse key.
  theta=min(math.pi/2,max(0.,abs(angle)))
  opening=math.sin(theta)**2 if theta>math.radians(.5)else 0.
  area=math.pi*.00125**2*opening
  # Water Cd=.62, edible oil reduced by an explicit viscosity surrogate.
  cd=.62 if self.water_ml>1e-9 else .22
  self.flow_ml_s=cd*area*math.sqrt(2*delta/density)*1e6
  transfer=min(self.water_ml+self.oil_ml,self.flow_ml_s*dt)
  water=min(self.water_ml,transfer);oil=transfer-water
  self.water_ml-=water;self.oil_ml-=oil
  if caught:self.cup_water_ml+=water;self.cup_oil_ml+=oil
  else:self.spill_ml+=transfer
  if not vented:self.p_air_Pa=self.sealed_pv/(GAS_EMPTY_ML-self.water_ml-self.oil_ml)
  if self.water_ml+self.oil_ml<1e-9:self.flow_ml_s=0.

 def snapshot(self):return asdict(self)
