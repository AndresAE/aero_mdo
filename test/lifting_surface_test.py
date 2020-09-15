from src.modeling import LiftingSurface
from test.test_library import is_close
wing = {
        'type': 'wing',
        'planform': 650,  # [ft^2]
        'aspect_ratio': 8,  # []
        'sweep_LE': 5,  # [deg]
        'taper': 0.5,  # []
        'station': 37.5,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 1,  # [ft]
        'c_l_alpha': 5,  # [1/rad]
        'airfoil_thickness': 0.15,  # []
}
horizontal = {
    'type': 'wing',
    'planform': 100,  # [ft^2]
    'aspect_ratio': 4,  # []
    'sweep_LE': 10,  # [deg]
    'taper': 0.465,  # []
    'station': 85,  # [ft]
    'buttline': 0,  # [ft]
    'waterline': 7,  # [ft]
    'c_l_alpha': 5,  # [1/rad]
    'airfoil_thickness': 0.1,  # []
}
vertical = {
    'planform': 50,  # [ft^2]
}
fuselage = {
    'width': 10,  # [ft]
    'height': 10,  # [ft]
}
mach = 0.3
altitude = 20000

w = LiftingSurface(wing)
c_l_a = w.c_l_alpha_wing(mach)
de_da = w.d_epsilon_d_alpha(horizontal, mach)
ac = w.aerodynamic_center()
cd0 = w.parasite_drag(mach, altitude)
ds_db_v = w.d_sigma_d_beta_vt(vertical, fuselage)
ds_db_h = w.d_sigma_d_beta_ht(horizontal, fuselage)

out = list()
out.append((is_close(c_l_a, 4.23295)))
out.append((is_close(de_da, 0.27033)))
out.append((is_close(ac, 41.2388)))
out.append((is_close(cd0, 0.00766999)))
out.append((is_close(ds_db_v, 0.95375)))
out.append((is_close(ds_db_h, 1.0715)))

if any(out):
    print("lifting surface test passed!")
else:
    print("lifting surface test failed")
