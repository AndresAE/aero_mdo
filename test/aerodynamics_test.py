from src.modeling.aerodynamics import dynamic_pressure, friction_coefficient, polhamus, reynolds_number
from test.test_library import is_close
from numpy import pi
mach = 0.3
altitude = 10000
c_l_alpha = 2 * pi
x_ref = 5
ar = 5
sweep_le = 30
taper = 0.5

q_e = dynamic_pressure(mach, altitude)
c_f = friction_coefficient(mach, altitude, x_ref)
re = reynolds_number(mach, altitude, x_ref)
c_l_alpha_w = polhamus(c_l_alpha, ar, mach, taper, sweep_le)

out = list()
out.append((is_close(q_e, 91.83224)))
out.append(is_close(c_f, 0.003111))
out.append(is_close(re, 8038249.384))
out.append(is_close(c_l_alpha_w, 4.2811))

mach = 0.6
altitude = 30000
ar = 3

q_e = dynamic_pressure(mach, altitude)
c_f = friction_coefficient(mach, altitude, x_ref)
re = reynolds_number(mach, altitude, x_ref)
c_l_alpha_w = polhamus(c_l_alpha, ar, mach, taper, sweep_le)

out.append((is_close(q_e, 158.89079)))
out.append(is_close(c_f, 0.00307937))
out.append(is_close(re, 8566969.2773))
out.append(is_close(c_l_alpha_w, 3.63125))

if any(out):
    print("aerodynamics test passed!")
else:
    print("aerodynamics wing test failed")
