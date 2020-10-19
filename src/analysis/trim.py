"""Trim aircraft longitudinally."""
from numpy import arcsin, array, cos, deg2rad, linalg, ones, rad2deg, sin, sqrt
from scipy.optimize import minimize, Bounds
from src.common import Atmosphere
from src.common.rotations import body_to_wind
from src.modeling.Aircraft import Aircraft
from src.modeling.force_model import c_f_m, landing_gear_loads
from src.modeling.trapezoidal_wing import mac, span
from src.modeling import Propulsion


# Linear Trims
def trim_aileron(aircraft, v, altitude, p):
    """trim aircraft with aileron and rudder"""
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    mach = v / a
    b = span(aircraft['wing']['aspect_ratio'], aircraft['wing']['planform'])
    k = b / (2 * v)
    c_l_p = Aircraft(aircraft, mach).c_r_roll_rate()  # []
    c_l_da = Aircraft(aircraft, mach).c_r_delta_aileron()  # []
    da = - c_l_p * p * k / c_l_da
    da = rad2deg(da)
    return da


def trim_aileron_rudder(aircraft, v, altitude, alpha, beta, p, r):
    """trim aircraft with aileron and rudder"""
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    mach = v / a
    b = span(aircraft['wing']['aspect_ratio'], aircraft['wing']['planform'])
    k = b / (2 * v)
    c_l_b = Aircraft(aircraft, mach).c_r_beta(alpha)
    c_l_p = Aircraft(aircraft, mach).c_r_roll_rate()  # []
    c_l_r = Aircraft(aircraft, mach).c_r_yaw_rate(alpha)  # []
    c_l_da = Aircraft(aircraft, mach).c_r_delta_aileron()  # []
    c_l_dr = Aircraft(aircraft, mach).c_r_delta_rudder()  # []
    c_n_b = Aircraft(aircraft, mach).c_n_beta(alpha)  # []
    c_n_p = Aircraft(aircraft, mach).c_n_roll_rate(alpha)  # []
    c_n_r = Aircraft(aircraft, mach).c_n_yaw_rate(alpha)  # []
    c_n_da = Aircraft(aircraft, mach).c_n_delta_aileron()  # []
    c_n_dr = Aircraft(aircraft, mach).c_n_delta_rudder()  # []
    a = array([[c_l_da, c_l_dr], [c_n_da, c_n_dr]])
    b = array([[-c_l_b * beta - c_l_p * p * k - c_l_r * r * k], [-c_n_b * beta - c_n_p * p * k - c_n_r * r * k]])
    c = linalg.solve(a, b)  # [rad]
    return rad2deg(c)


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


def trim_vs(aircraft, altitude, gamma, n=1):
    """trim aircraft with angle of attack and elevator"""
    rho = Atmosphere(altitude).air_density()  # [slug / ft^3]
    mach = 0.3  # [] assume moderate mach number
    c_l_a = Aircraft(aircraft, mach).c_l_alpha()  # [1/rad]
    c_l_de = Aircraft(aircraft, mach).c_l_delta_elevator()  # [1/rad]
    c_m_a = Aircraft(aircraft, mach).c_m_alpha()  # [1/rad]
    c_m_de = Aircraft(aircraft, mach).c_m_delta_elevator()  # [1/rad]
    c_l_0 = Aircraft(aircraft, mach).c_l_zero()  # []
    w = aircraft['weight']['weight'] * n  # [lb]
    s_w = aircraft['wing']['planform']  # [ft^2]
    a_s = deg2rad(aircraft['wing']['alpha_stall'])
    c_m_0 = Aircraft(aircraft, mach).c_m_zero(altitude)
    a = array([[- w * cos(deg2rad(gamma)) / (0.5 * rho * s_w), c_l_de], [0, c_m_de]])
    b = array([[-c_l_0 - c_l_a * a_s], [- c_m_0 - c_m_a * a_s]])
    c = linalg.solve(a, b)  # [rad]
    v_s = float((1 / c[0]) ** 0.5)
    return v_s


# Nonlinear trims
def trim_aileron_nonlinear(aircraft, speed, altitude, roll_rate, tol=1e-1):
    """trim with aileron, nonlinear."""
    def obj(x):
        out = abs(x)
        return out

    def aileron(x):
        u = array([x[0], 0, 0, 0.01])
        x = array([speed, 0, 0, 0, 0, 0, roll_rate, 0, 0, 0, 0, altitude])
        dxdt = c_f_m(aircraft, x, u)
        return sqrt(sum(dxdt ** 2)) / 1000

    lim_ail = aircraft['wing']['control_1']['limits']
    lim = Bounds(deg2rad(lim_ail[0]), deg2rad(lim_ail[1]))
    x0 = array([0.0])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': aileron}),
                     options=({'maxiter': 200}))
    c = rad2deg(u_out['x'])
    return c


def trim_aileron_rudder_nonlinear(aircraft, speed, altitude, alpha, beta, roll_rate, yaw_rate, tol=1e-1):
    """trim with aileron and rudder, nonlinear."""
    def obj(x):
        out = sum(abs(x))
        return out

    def aileron_rudder(x):
        u = array([x[0], 0, x[1], 0.01])
        b2w = body_to_wind(alpha, beta)
        v_b = linalg.inv(b2w) @ array([speed, 0, 0])
        x = array([v_b[0], v_b[1], v_b[2], 0, alpha, 0, roll_rate, 0, yaw_rate, 0, 0, altitude])
        dxdt = c_f_m(aircraft, x, u)
        return sqrt(sum(dxdt ** 2)) / 1000

    lim_ail = aircraft['wing']['control_1']['limits']
    lim_rud = aircraft['vertical']['control_1']['limits']
    lim = ([deg2rad(lim_ail[0]), deg2rad(lim_ail[1])], [deg2rad(lim_rud[0]), deg2rad(lim_rud[1])])
    x0 = array([0.0, 0.0])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': aileron_rudder}),
                     options=({'maxiter': 200}))
    c = rad2deg(u_out['x'])
    return c


def trim_aileron_rudder_speed_nonlinear(aircraft, altitude, beta, roll_rate, yaw_rate, tol=1e-1):
    """trim with aileron and rudder, nonlinear."""
    def obj(x):
        out = sum(abs(x))
        return out

    def aileron_rudder_speed(x):
        u = array([x[0], x[3], x[1], 1])
        alpha = x[2]
        b2w = body_to_wind(alpha, beta)
        v_b = linalg.inv(b2w) @ array([x[4], 0, 0])
        x = array([v_b[0], v_b[1], v_b[2], 0, alpha, 0, roll_rate, 0, yaw_rate, 0, 0, altitude])
        cfm = c_f_m(aircraft, x, u)
        return cfm[1] + cfm[2] + cfm[3] + cfm[4] + cfm[5]

    lim_ail = aircraft['wing']['control_1']['limits']
    lim_rud = aircraft['vertical']['control_1']['limits']
    lim_ele = aircraft['horizontal']['control_1']['limits']
    lim = ([deg2rad(lim_ail[0]), deg2rad(lim_ail[1])], [deg2rad(lim_rud[0]), deg2rad(lim_rud[1])],
           [-5/57.3, aircraft['wing']['alpha_stall']/57.3], [deg2rad(lim_ele[0]), deg2rad(lim_ele[1])], [10, 500])
    x0 = array([0.0, 0.0, 0.0, 0.0, 500])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': aileron_rudder_speed}),
                     options=({'maxiter': 200}))
    c = u_out['x']
    c[0:4] = rad2deg(c[0:4])
    return c


def trim_alpha_de_nonlinear(aircraft, speed, altitude, gamma, n=1, tol=1e-1):
    """trim nonlinear aircraft with angle of attack and elevator."""
    w_in = aircraft['weight']['weight']
    aircraft['weight']['weight'] = w_in * n

    def obj(x):
        out = x[1]
        return out

    def alpha_stab(x):
        u = array([0, x[1], 0, 0.01])
        x = array([speed * cos(x[0]), 0, speed * sin(x[0]), 0, x[0] + deg2rad(gamma), 0, 0, 0, 0, 0, 0, altitude])
        cfm = c_f_m(aircraft, x, u)
        return abs(cfm[2]) + abs(cfm[4])

    lim_ele = aircraft['horizontal']['control_2']['limits']
    lim = ([-5/57.3, aircraft['wing']['alpha_stall']/57.3], [deg2rad(lim_ele[0]), deg2rad(lim_ele[1])])
    x0 = array([0.01, -0.01])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': alpha_stab}),
                     options=({'maxiter': 400}))
    c = rad2deg(u_out['x'])
    aircraft['weight']['weight'] = w_in
    return c


def trim_vr(aircraft, altitude, u_0, tol=1e-1):
    """return nose unstick speed."""
    def obj(x):
        out = x[0]
        return out

    def v_stab(x):
        x = array([x[0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, altitude])
        c = c_f_m(aircraft, x, u_0)
        c_t, c_g, normal_loads = landing_gear_loads(aircraft, x, c)
        return float(normal_loads[0])

    lim = Bounds(10, 500)
    x0 = array([10])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': v_stab}),
                     options=({'maxiter': 200}))
    return float(u_out['x'])


def trim_vs_nonlinear(aircraft, altitude, alpha, gamma, n=1, tol=1e-1):
    """trim nonlinear aircraft with speed and elevator."""
    alpha = deg2rad(alpha)
    w_in = aircraft['weight']['weight']
    aircraft['weight']['weight'] = w_in * n

    def obj(x):
        out = x[1]
        return out

    def v_stab(x):
        u = array([0, x[0], 0, 0.01])
        x_in = array([x[1] * cos(alpha), 0, x[1] * sin(alpha), 0, alpha + deg2rad(gamma), 0, 0, 0, 0, 0, 0, altitude])
        cfm = c_f_m(aircraft, x_in, u)
        return abs(cfm[2]) + abs(cfm[4])

    lim_ele = aircraft['horizontal']['control_2']['limits']
    lim = ([deg2rad(lim_ele[0]), deg2rad(lim_ele[1])], [10, 500])
    x0 = array([deg2rad(lim_ele[0]), 500])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': v_stab}),
                     options=({'maxiter': 500}))
    c = u_out['x']
    c[0] = rad2deg(u_out['x'][0])
    aircraft['weight']['weight'] = w_in
    return c


def trim_vx(aircraft, altitude, tol=1e-1):
    """trim nonlinear aircraft to best climb angle."""
    th = 1

    def obj(x):
        u = array([0, x[1], 0, th])
        speed = x[2]
        x = array([speed * cos(x[0]), 0, speed * sin(x[0]), 0, x[0], 0, 0, 0, 0, 0, 0, altitude])
        c_1 = c_f_m(aircraft, x, u)
        p_s = (c_1[0]) * speed / aircraft['weight']['weight']  # [ft/min]
        gamma = arcsin(p_s / speed)
        return - gamma

    def alpha_stab(x):
        u = array([0, x[1], 0, th])
        speed = x[2]
        x = array([speed * cos(x[0]), 0, speed * sin(x[0]), 0, x[0], 0, 0, 0, 0, 0, 0, altitude])
        cfm = c_f_m(aircraft, x, u)
        return abs(cfm[2]) + abs(cfm[4])

    lim_ele = aircraft['horizontal']['control_2']['limits']
    lim = ([-5/57.3, aircraft['wing']['alpha_stall']/57.3], [deg2rad(lim_ele[0]), deg2rad(lim_ele[1])], [0.1, 1000])
    x0 = array([0.01, -0.01, 100])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': alpha_stab}),
                     options=({'maxiter': 500}))
    c = u_out['x']
    c[0:2] = rad2deg(c[0:2])
    return c


def trim_vy(aircraft, altitude, tol=1e-1):
    """trim nonlinear aircraft to best rate of climb."""
    th = 1

    def obj(x):
        u = array([0, x[1], 0, th])
        speed = x[2]
        x = array([speed * cos(x[0]), 0, speed * sin(x[0]), 0, x[0], 0, 0, 0, 0, 0, 0, altitude])
        c_1 = c_f_m(aircraft, x, u)
        p_s = (c_1[0]) * speed * 60 / aircraft['weight']['weight']  # [ft/min]
        return - p_s

    def alpha_stab(x):
        u = array([0, x[1], 0, th])
        speed = x[2]
        x = array([speed * cos(x[0]), 0, speed * sin(x[0]), 0, x[0], 0, 0, 0, 0, 0, 0, altitude])
        cfm = c_f_m(aircraft, x, u)
        return abs(cfm[2]) + abs(cfm[4])

    lim_ele = aircraft['horizontal']['control_2']['limits']
    lim = ([-5/57.3, aircraft['wing']['alpha_stall']/57.3], [deg2rad(lim_ele[0]), deg2rad(lim_ele[1])], [0.1, 1000])
    x0 = array([0.01, -0.01, 20])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': alpha_stab}),
                     options=({'maxiter': 200}))
    c = u_out['x']
    c[0:2] = rad2deg(c[0:2])
    return c
