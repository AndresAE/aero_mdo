from src.modeling.flap import c_f_m_flap
from test.test_library import is_close
ht = {
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
    'control_1': {
            'name': 'elevator_l',
            'cf_c': 0.3,
            'b_1': -0.999,
            'b_2': -0.01,
        },
    'control_2': {
        'name': 'elevator_r',
        'cf_c': 0.3,
        'b_1': 0.01,
        'b_2': 0.999,
    },
}
cg = [40, 0, 1]
mach = 0.5

c_l = c_f_m_flap(ht, ht['control_1'], mach, cg)
c_r = c_f_m_flap(ht, ht['control_2'], mach, cg)
c = c_l + c_r

out = list()
out.append((is_close(c_l[3], 6.1968)))
out.append((is_close(c_r[3], -6.1968)))
out.append((is_close(c[1], 0)))
out.append((is_close(c[2], -1.0188)))
out.append((is_close(c[4], -49.36)))

if any(out):
    print("flap test passed!")
else:
    print("flap test failed")
