"""Trim aircraft longitudinally."""
from numpy import array, cos, deg2rad, interp, linalg, ones, rad2deg, sin, sqrt
from scipy.optimize import minimize
from src.common import Atmosphere
from src.modeling.Aircraft import Aircraft
from src.modeling.force_model import c_f_m, landing_gear_loads
from src.modeling.trapezoidal_wing import mac
from src.modeling import Propulsion


# Linear Trims
def trim_alpha_de(aircraft, speed, altitude, gamma, n=1):
    """trim aircraft with angle of attack and elevator"""
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    rho = Atmosphere(altitude).air_density()  # [slug / ft^3]
    mach = speed / a  # []
    c_l_a = Aircraft(aircraft, mach).c_l_alpha()  # [1/rad]
    c_l_de = Aircraft(aircraft, mach).c_l_delta_elevator()  # [1/rad]
    c_m_a = Aircraft(aircraft, mach).c_m_alpha()  # [1/rad]
    c_m_de = Aircraft(aircraft, mach).c_m_delta_elevator()  # [1/rad]
    c_l_0 = Aircraft(aircraft, mach).c_l_zero()  # []
    a = array([[c_l_a, c_l_de], [c_m_a, c_m_de]])
    w = aircraft['weight']['weight']*n  # [lb]
    q_bar = 0.5 * rho * speed ** 2  # [psf]
    s_w = aircraft['wing']['planform']  # [ft^2]
    c_l_1 = w * cos(deg2rad(gamma)) / (s_w * q_bar)  # []
    c_m_0 = Aircraft(aircraft, mach).c_m_zero(altitude)
    b = array([[c_l_1 - c_l_0], [- c_m_0]])
    c = linalg.solve(a, b)  # [rad]
    return rad2deg(c)


def trim_alpha_de_throttle(aircraft, speed, altitude, gamma, n=1):
    """trim aircraft with angle of attack, elevator, and throttle"""
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    rho = Atmosphere(altitude).air_density()  # [slug / ft^3]
    mach = speed / a  # []
    c_l_a = Aircraft(aircraft, mach).c_l_alpha()  # [1/rad]
    c_l_de = Aircraft(aircraft, mach).c_l_delta_elevator()  # [1/rad]
    c_m_a = Aircraft(aircraft, mach).c_m_alpha()  # [1/rad]
    c_m_de = Aircraft(aircraft, mach).c_m_delta_elevator()  # [1/rad]
    c_l_0 = Aircraft(aircraft, mach).c_l_zero()  # []
    c_d_0 = Aircraft(aircraft, mach).c_d_zero(altitude)  # []
    w = aircraft['weight']['weight']*n  # [lb]
    q_bar = 0.5 * rho * speed ** 2  # [psf]
    s_w = aircraft['wing']['planform']  # [ft^2]
    c_bar = mac(aircraft['wing']['aspect_ratio'], s_w, aircraft['wing']['taper'])
    c_l_1 = w * cos(deg2rad(gamma)) / (s_w * q_bar)  # []
    t = Propulsion(aircraft['propulsion'], [speed, altitude],
                   ones((aircraft['propulsion']['n_engines'])), aircraft['weight']['cg']).thrust_f_m()
    a = array([[-c_l_a, -c_l_de, 0],
               [c_m_a, c_m_de, (t[4]) / (s_w * q_bar * c_bar)],
               [-2 * c_l_1, 0, t[0] / (s_w * q_bar)]])
    c_m_0 = Aircraft(aircraft, mach).c_m_zero(altitude)
    b = array([[-c_l_1 + c_l_0], [- c_m_0], [w * sin(deg2rad(gamma)) / (s_w * q_bar) + c_d_0]])
    c = linalg.solve(a, b)  # [rad]
    c[0:2] = rad2deg(c[0:2])
    return c


def trim_vr(aircraft, u_0):
    v = [5 * x for x in range(1, 100)]
    out = []
    for vi in v:
        x = array([vi, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        c = c_f_m(aircraft, x, u_0)
        c_t, c_g, normal_loads = landing_gear_loads(aircraft, x, c)
        out.append(float(normal_loads[0]))
    v_r = interp(0, out, v)
    return v_r


# Optimization Trims
def trim(aircraft, speed, altitude, gamma, n=1, tol=1e-1):
    # automate limits
    # add nonzero derivatives
    # make generic
    lim = ([-80/57.3, 80/57.3], [0.01, 1], [-80/57.3, 80/57.3])
    x0 = array([-0.001, 0.4, 0.001])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': alpha_stab_throttle,
                                  'args': (aircraft, speed, altitude, gamma, n)}),
                     options=({'maxiter': 200}))
    return u_out


def obj(x):
    out = sum(abs(x))
    return out


def alpha_stab_throttle(x, aircraft, speed, altitude, gamma, n):
    u = array([0, x[0], 0, x[1]])
    x = array([speed * cos(x[2]), 0, speed * sin(x[2]), 0, x[2] + deg2rad(gamma), 0, 0, 0, 0, 0, 0, altitude])
    aircraft['weight']['weight'] = aircraft['weight']['weight'] * n
    dxdt = c_f_m(aircraft, x, u)
    return sqrt(sum(dxdt ** 2)) / 1000
