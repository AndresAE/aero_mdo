"""Trim aircraft longitudinally."""
from numpy import array, concatenate, cos, deg2rad, interp, isnan, linalg, rad2deg
from src.common import Atmosphere
from src.modeling.Aircraft import Aircraft
from src.modeling.force_model import c_f_m, landing_gear_loads
# from src.common.equations_of_motion import nonlinear_eom_to_ss
# from src.common import Earth


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


# def trim_alpha_de_throttle(aircraft, speed, altitude, gamma):
#     """trim aircraft with angle of attack, elevator, and throttle"""
#     a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
#     rho = Atmosphere(altitude).air_density()  # [slug / ft^3]
#     mach = speed / a  # []
#     c_l_a = Aircraft(aircraft, mach).c_l_alpha()  # [1/rad]
#     c_l_de = Aircraft(aircraft, mach).c_l_delta_elevator()  # [1/rad]
#     c_m_a = Aircraft(aircraft, mach).c_m_alpha()  # [1/rad]
#     c_m_de = Aircraft(aircraft, mach).c_m_delta_elevator()  # [1/rad]
#     c_l_0 = Aircraft(aircraft, mach).c_l_zero()  # []
#     a = array([[c_l_a, c_l_de], [c_m_a, c_m_de]])
#     w = aircraft['weight']['weight']  # [lb]
#     q_bar = 0.5 * rho * speed ** 2  # [psf]
#     s_w = aircraft['wing']['planform']  # [ft^2]
#     c_l_1 = w * cos(deg2rad(gamma)) / (s_w * q_bar)  # []
#     c_m_0 = Aircraft(aircraft, mach).c_m_zero(altitude)
#     b = array([[c_l_1 - c_l_0], [- c_m_0]])
#     c = linalg.solve(a, b)  # [rad]
#     return rad2deg(c)


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


# def trim(aircraft, x_dot_known, x_known, u_0, eqn, con, tol=1e-3):
#     g = Earth(x_known[-1]).gravity()
#     m = aircraft['weight']['weight'] / g
#     j = aircraft['weight']['inertia']
#
#     x_ss = array(range(0, 12))
#
#     x_dot_out = array(range(0, len(eqn)))
#     unknown_flag = isnan(x_known)
#     x_known[unknown_flag] = 0
#     u_out = []
#     s_out = []
#     while abs(sum(x_dot_out - x_dot_known[eqn])) > tol:
#         a, b, c, d = nonlinear_eom_to_ss(aircraft, x_ss, con, x_known, u_0, m, j)
#         a_trim = a[:, eqn]
#         a_trim = a_trim[eqn, :]
#         b_trim = a[:, unknown_flag]
#         b_trim = b_trim[eqn, :]
#         b_u = b[eqn, :]
#         b_trim = concatenate((b_trim, b_u), axis=1)
#         u = linalg.inv(b_trim) @ (x_dot_known[eqn] - a_trim @ x_known[eqn])
#         u_out = u[-len(con) - 1:-1]
#         s_out = u[0:-len(con)]
#         x_known[unknown_flag] = s_out
#         u_0[con] = u_out
#         a_out, b_out, c, d = nonlinear_eom_to_ss(aircraft, eqn, con, x_known, u_0, m, j)
#         x_dot_out = a_out @ x_known[eqn] + b_out @ u_out
#         print(u_out)
#         print(abs(sum(x_dot_out - x_dot_known[eqn])))
#     return s_out, u_out
