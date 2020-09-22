from control import damp, StateSpace
from matplotlib import pyplot as plt
from numpy import array, max, min, unique, pi, sum, sqrt, zeros
from src.analysis.trim import trim_alpha_de_throttle, trim_vr
from src.common import Atmosphere, Earth
from src.common.equations_of_motion import nonlinear_eom, nonlinear_eom_to_ss
from src.modeling.Aircraft import Aircraft
from src.modeling.force_model import c_f_m, landing_gear_loads
g = Earth(0).gravity()  # f/s2


def static_margin(plane, mach):
    """return longitudinal static margin."""
    c_m_a = Aircraft(plane, mach).c_m_alpha()
    c_l_a = Aircraft(plane, mach).c_l_alpha()
    sm = -c_m_a / c_l_a * 100
    return sm


def n_per_alpha(aircraft, x_0):
    """return acceleration sensitivity."""
    rho = Atmosphere(x_0[-1]).air_density()  # [slug/ft3]
    a = Atmosphere(x_0[-1]).speed_of_sound()  # [ft/s]
    v = sqrt(sum(x_0[0:3]**2))  # [ft/s]
    q_bar = 0.5*rho*v**2  # [psf]
    s = aircraft['wing']['planform']  # [ft2]
    cla = Aircraft(aircraft, v / a).c_l_alpha()  # [1/rad]
    n_a = cla*s*q_bar/aircraft['weight']['weight']  # [g/rad]
    return n_a


def short_period_mode(aircraft, x_0, u_0):
    """return short-period modal criteria."""
    wn_sp, zeta_sp, wn_ph, zeta_ph = long_modes(aircraft, x_0, u_0)
    n_a = n_per_alpha(aircraft, x_0)  # [g/rad]
    cap = (wn_sp**2)/n_a
    return wn_sp, zeta_sp, cap


def phugoid_mode(aircraft, x_0, u_0):
    """return phugoid modal criteria."""
    wn_sp, zeta_sp, wn_ph, zeta_ph = long_modes(aircraft, x_0, u_0)
    return wn_ph, zeta_ph


def long_modes(aircraft, x_0, u_0):
    """longitudinal mode calculations."""
    j = aircraft['weight']['inertia']
    m = aircraft['weight']['weight'] / g  # [slug]
    x_ss = [0, 2, 4, 7]
    a, b, c, d = nonlinear_eom_to_ss(aircraft, x_ss, [1], x_0, u_0, m, j)
    sys = StateSpace(a, b, c, d)
    wn, zeta, poles = damp(sys)
    wn_sp = unique(max(wn))
    zeta_sp = unique(zeta[wn == wn_sp])
    wn_ph = unique(min(wn))
    zeta_ph = unique(zeta[wn == wn_ph])
    return wn_sp, zeta_sp, wn_ph, zeta_ph


def plot_sp():
    """short-period requirement patches."""
    x_lvl_1 = [0.35, 1.3, 1.3, 0.35, 0.35]
    y_lvl_1 = [3.6, 3.6, 0.16, 0.16, 3.6]
    x_lvl_2 = [0.25, 2, 2, 0.25, 0.25]
    y_lvl_2 = [10, 10, 0.05, 0.05, 10]
    x_lvl_3 = [0.15, 0.15]
    y_lvl_3 = [0, 12]
    plt.plot(x_lvl_1, y_lvl_1, 'k')
    plt.plot(x_lvl_2, y_lvl_2, 'k')
    plt.plot(x_lvl_3, y_lvl_3, 'k')
    plt.xlim((0.01, 2.5))
    plt.ylim((0.01, 12))
    plt.ylabel('Control Anticipation Parameter [1/g*s2]')
    plt.xlabel('Short-Period Damping Ratio')
    plt.yscale("log")


def specific_excess_power(aircraft, x, u):
    """calculate specific excess power"""
    w = aircraft['weight']['weight']  # [lbs]
    v = (x[0]**2+x[2]**2)**0.5  # [ft/s]
    c = c_f_m(aircraft, x, u)
    p_s = (c[0]) * v * 60 / w  # [ft/min]
    return p_s


def l_over_d(aircraft, x, u):
    """calculate specific excess power"""
    w = aircraft['weight']['weight']  # [lbs]
    v = (x[0]**2+x[2]**2)**0.5  # [ft/s]
    c = c_f_m(aircraft, x, u)
    p_s = (c[0]) * v * 60 / w  # [ft/min]
    return p_s


def maneuvering(aircraft, mach, altitude, n_z):
    """return trim parameters for given n_z vector."""
    v = Atmosphere(altitude).speed_of_sound() * mach
    alpha_out = []
    de_out = []
    for ni in n_z:
        out = trim_alpha_de_throttle(aircraft, v, altitude, 0, ni)
        alpha_out.append(out[0])
        de_out.append(out[1])
    return alpha_out, de_out


def balanced_field_length(aircraft, x_0, u_0, rotate_margin=1):
    """return balanced field length."""
    v_rotate = trim_vr(aircraft, u_0)
    dt = 0.01
    v = []
    s = []
    h = []
    pitch = []
    de = []
    x = x_0
    t = 0
    t_out = []
    p = zeros(len(x_0))
    while x[0] < v_rotate:
        dxdt = takeoff_ground_roll(aircraft, x, u_0)
        p = p + x*dt + 0.5*dxdt*dt**2
        x = x + dxdt * dt
        v.append(x[0])
        s.append(p[0])
        h.append(-x[-1])
        pitch.append(x[4]*180/pi)
        de.append(u_0[1]*180/pi)
        t = t + dt
        t_out.append(t)

    u_0[1] = -20 * pi / 180
    while pitch[-1] < aircraft['wing']['alpha_stall'] - rotate_margin:
        dxdt = takeoff_ground_roll(aircraft, x, u_0)
        p = p + x * dt + 0.5 * dxdt * dt ** 2
        x = x + dxdt * dt
        v.append(x[0])
        s.append(p[0])
        h.append(-x[-1])
        pitch.append(x[4] * 180 / pi)
        de.append(u_0[1] * 180 / pi)
        t = t + dt
        t_out.append(t)

    out = trim_alpha_de_throttle(aircraft, v[-1], h[-1], 10)
    u_0[1] = -5 * pi / 180
    while h[-1] < 30:
        dxdt = takeoff_ground_roll(aircraft, x, u_0)
        p = p + x * dt + 0.5 * dxdt * dt ** 2
        x = x + dxdt * dt
        v.append(x[0])
        s.append(p[0])
        h.append(-x[-1])
        pitch.append(x[4] * 180 / pi)
        de.append(u_0[1] * 180 / pi)
        t = t + dt
        t_out.append(t)

    plt.subplot(4, 1, 1)
    plt.plot(t_out, s)
    plt.subplot(4, 1, 2)
    plt.plot(t_out, v)
    plt.subplot(4, 1, 3)
    plt.plot(t_out, h)
    plt.subplot(4, 1, 4)
    plt.plot(t_out, pitch)
    plt.plot(t_out, de)
    plt.show()


def takeoff_ground_roll(aircraft, x_0, u_0):
    """return derivatives for aircraft on ground."""
    m = aircraft['weight']['weight']/g
    j = aircraft['weight']['inertia']
    c = c_f_m(aircraft, x_0, u_0)
    c_t, c_g, normal_loads = landing_gear_loads(aircraft, x_0, c, True)
    dxdt = nonlinear_eom(x_0, m, j, c_t)
    return dxdt
