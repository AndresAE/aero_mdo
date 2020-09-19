from src.airplanes.plane_1 import plane as aircraft
from src.analysis.trim import trim_vr
from numpy import array

x = array([float(200), 0, float("NaN"), 0, float("NaN"), 0, 0, 0, 0, 0, 0, float(5000)])
x_dot = array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = array([0, -30/57.3, 0, 1])
c = trim_vr(aircraft, u0)
print(c)
