from src.airplanes.plane_1 import plane as aircraft
from src.modeling.force_model import c_f_m
from numpy import array

x = array([float(200), 0, float(10), 0, float(5/57.3), 0, 0, 0, 0, 0, 0, float(5000)])
throttle = array([1, 1, 1, 1, 1, 1]) * 1
cfm = c_f_m(aircraft, x, [0, -6.6/57.3, 0], throttle)
print(cfm)


