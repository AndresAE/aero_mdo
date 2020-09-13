from common.atmosphere import air_density
from numpy import array, cos as c, cross, deg2rad, sin as s, sum, zeros
from scipy.interpolate import RectBivariateSpline


def thrust_f_m(aircraft, x, throttle):
    """returns total propulsion forces and moments."""
    cg = aircraft['weight']['cg']
    c_f_m = zeros((aircraft['propulsion']['n_engines'], 6))
    for ii in range(0, aircraft['propulsion']['n_engines']):
        engine = aircraft['propulsion']["engine_%d" % (ii + 1)]
        if engine['type'] == 'jet':
            c_f_m[ii, :] = jet_engine(engine, cg, x[-1], throttle[ii])
        elif engine['type'] == 'prop':
            c_f_m[ii, :] = propeller(engine, cg, x[0], throttle[ii])
    c_f_m = sum(c_f_m, axis=0)
    return c_f_m


def jet_engine(engine, cg, altitude, throttle):
    """returns jet engine forces and moments."""
    rho = air_density(altitude)
    rho_sl = air_density(0)
    t_slo = engine['thrust']*throttle
    phi = deg2rad(engine['thrust_angle'])
    psi = deg2rad(engine['toe_angle'])
    r = array([-engine['station'], engine['buttline'], -engine['waterline']])+array(cg)
    t = t_slo*rho/rho_sl  # [lbs]
    t = t*array([c(phi)*c(psi), s(psi), -s(phi)])
    m = cross(r, t)
    c_f_m = array([t[0], t[1], t[2], m[0], m[1], m[2]])
    return c_f_m


def propeller(engine, cg, v, throttle):
    """returns propeller system forces and moments."""
    # thrust coefficient table
    c_t = array([[0.14, 0.08, 0, -0.07, -0.15, -0.21],
                 [0.16, 0.15, 0.1, 0.02, -0.052, -0.1],
                 [0.18, 0.172, 0.16, 0.12, 0.04, -0.025]])
    pitch_c_t = [15, 25, 35]
    j_c_t = [0, 0.4, 0.8, 1.2, 1.6, 2.0]

    rpm = engine['rpm_max'] * throttle / 60
    j = array([v/(engine['diameter'] * rpm)])
    f = RectBivariateSpline(pitch_c_t, j_c_t, c_t, kx=1)
    c_t_i = f(engine['pitch'], j)
    rho = air_density(0)
    t = float(rho * (rpm ** 2) * (engine['diameter'] ** 4) * c_t_i)
    phi = deg2rad(engine['thrust_angle'])
    psi = deg2rad(engine['toe_angle'])
    r = array([-engine['station'], engine['buttline'], -engine['waterline']]) + array(cg)
    t = t * array([c(phi) * c(psi), s(psi), -s(phi)])
    m = cross(r, t)
    c_f_m = array([t[0], t[1], t[2], m[0], m[1], m[2]])
    return c_f_m
