from src.modeling import Propulsion
from test.test_library import is_close

propulsion = {
        'n_engines': 2,
        'engine_1': {
            'type': 'prop',
            'station': 30,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': 30,  # [ft]
            'thrust_angle': 5,  # [deg]
            'toe_angle': 2,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 10000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_2': {
            'type': 'prop',
            'station': 30,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': -30,  # [ft]
            'thrust_angle': 5,  # [deg]
            'toe_angle': -2,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 10000,  # [rpm]
            'pitch': 20  # [deg]
        },
}
cg = [43, 0, 2]
throttle = [1, 1]
x = [100, 30000]
p = Propulsion(propulsion, x, throttle, cg)
cfm = p.thrust_f_m()

out = list()
out.append((is_close(cfm[0], 4780.1585)))
out.append((is_close(cfm[4], 10220.1982)))
out.append((is_close(cfm[3], 0)))

throttle = [0.5, 0.5]
x = [200, 0]
p = Propulsion(propulsion, x, throttle, cg)
cfm = p.thrust_f_m()
out.append((is_close(cfm[0], 713.781)))
out.append((is_close(cfm[4], 1526.0972)))
out.append((is_close(cfm[3], 0)))

propulsion = {
        'n_engines': 1,
        'engine_1': {
            'type': 'jet',
            'station': 30,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': 0,  # [ft]
            'thrust_angle': 5,  # [deg]
            'toe_angle': -2,  # [deg]
            'thrust': 500,  # [lbs]
        }
}
throttle = [0.5]
p = Propulsion(propulsion, x, throttle, cg)
cfm = p.thrust_f_m()
out.append((is_close(cfm[0], 248.896)))
out.append((is_close(cfm[4], 532.1531)))
out.append((is_close(cfm[3], 8.7248)))

throttle = [1, 1]
x = [200, 30000]
p = Propulsion(propulsion, x, throttle, cg)
cfm = p.thrust_f_m()
out.append((is_close(cfm[0], 186.60355)))
out.append((is_close(cfm[4], 398.9669)))
out.append((is_close(cfm[3], 6.5412)))

if any(out):
    print("propulsion test passed!")
else:
    print("propulsion test failed")
