from src.airplanes.plane_1 import plane as aircraft
from src.modeling.flap import c_f_m_flap

cg = aircraft['weight']['cg']
w = aircraft['wing']
c = aircraft['wing']['control_4']
m = 0.3
c = c_f_m_flap(w, c, m, cg)
print(c)


