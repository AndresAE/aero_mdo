from control import damp, StateSpace
from matplotlib import pyplot as plt
from numpy import log, max, min, unique, pi, sum, sqrt, zeros
from src.analysis.trim import trim_alpha_de_throttle, trim_vr, trim_vs
from src.common import Atmosphere, Earth
from src.common.equations_of_motion import nonlinear_eom, nonlinear_eom_to_ss
from src.modeling.Aircraft import Aircraft
from src.modeling.force_model import c_f_m, landing_gear_loads
g = Earth(0).gravity()  # f/s2


def aircraft_range(aircraft, mach, altitude):
    """return aircraft range in nautical miles."""
    sigma = aircraft['propulsion']['energy_density']
    eta = aircraft['propulsion']['total_efficiency']
    m_fuel = aircraft['propulsion']['fuel_mass']
    l_d = l_over_d(aircraft, mach, altitude)
    w_i = aircraft['weight']['weight']
    if aircraft['propulsion']['const_mass']:
        r = (sigma * eta * m_fuel) / (w_i / l_d) / 6076
    else:
        w_f = w_i - m_fuel * g
        r = sigma / g * l_d * eta * log(w_i / w_f) / 6076
    return r


def balanced_field_length(aircraft, x_0, u_0, rotate_margin=1, h_f=35):
    """return balanced field length."""
    alt_f = x_0[-1] + h_f
    u_0[1] = aircraft['horizontal']['control_1']['limits'][0] * pi / 180
    v_unstick = trim_vr(aircraft, x_0[-1], u_0)
    v_s = trim_vs(aircraft, x_0[-1], 0)
    v_rotate = 1.2 * max([v_s, v_unstick])
    u_0[1] = 0.0
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
        h.append(x[-1])
        pitch.append(x[4]*180/pi)
        de.append(u_0[1]*180/pi)
        t = t + dt
        t_out.append(t)
    x_rto = x
    u_0[1] = aircraft['horizontal']['control_1']['limits'][0] * pi / 180
    while pitch[-1] < aircraft['wing']['alpha_stall'] - rotate_margin:
        dxdt = takeoff_ground_roll(aircraft, x, u_0)
        p = p + x * dt + 0.5 * dxdt * dt ** 2
        x = x + dxdt * dt
        v.append(x[0])
        s.append(p[0])
        h.append(x[-1])
        pitch.append(x[4] * 180 / pi)
        de.append(u_0[1] * 180 / pi)
        t = t + dt
        t_out.append(t)

    out = trim_alpha_de_throttle(aircraft, v[-1], h[-1], 5)
    u_0[1] = out[1] * pi / 180
    u_0[3] = out[2]
    while h[-1] < alt_f:
        dxdt = takeoff_ground_roll(aircraft, x, u_0)
        p = p + x * dt + 0.5 * dxdt * dt ** 2
        x = x + dxdt * dt
        v.append(x[0])
        s.append(p[0])
        h.append(x[-1])
        pitch.append(x[4] * 180 / pi)
        de.append(u_0[1] * 180 / pi)
        t = t + dt
        t_out.append(t)
    u_rto = u_0
    u_rto[-1] = 0.001
    v_rto, s_rto = rejected_takeoff(aircraft, s[-1], x_rto, u_rto)

    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(s, v)
    plt.plot(s_rto, v_rto, 'r')
    plt.ylabel('v [ft/s]')
    plt.ylim((0, max(v) + 20))
    plt.xlim((0, s[-1] + 10))
    plt.grid(True)
    plt.subplot(3, 1, 2)
    plt.plot(s, h)
    plt.ylabel('h [ft]')
    plt.xlim((0, s[-1] + 10))
    plt.grid(True)
    plt.subplot(3, 1, 3)
    plt.plot(s, pitch, label='pitch')
    plt.plot(s, de, label='Elevator')
    plt.legend()
    plt.ylabel('angles [deg]')
    plt.xlabel('distance [ft]')
    plt.xlim((0, s[-1] + 10))
    plt.grid(True)


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


def l_over_d(aircraft, mach, altitude):
    """calculate specific excess power"""
    c_d_0 = Aircraft(aircraft, mach).c_d_zero(altitude)
    c_l_a = Aircraft(aircraft, mach).c_l_alpha()
    ar = aircraft['wing']['aspect_ratio']
    l_d = 1 / 2 * (((c_l_a / 2) * ar) / c_d_0) ** 0.5
    return l_d


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


def phugoid_mode(aircraft, x_0, u_0):
    """return phugoid modal criteria."""
    wn_sp, zeta_sp, wn_ph, zeta_ph = long_modes(aircraft, x_0, u_0)
    return wn_ph, zeta_ph


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


def short_period_mode(aircraft, x_0, u_0):
    """return short-period modal criteria."""
    wn_sp, zeta_sp, wn_ph, zeta_ph = long_modes(aircraft, x_0, u_0)
    n_a = n_per_alpha(aircraft, x_0)  # [g/rad]
    cap = (wn_sp**2)/n_a
    return wn_sp, zeta_sp, cap


def specific_excess_power(aircraft, x, u):
    """calculate specific excess power"""
    w = aircraft['weight']['weight']  # [lbs]
    v = (x[0]**2+x[2]**2)**0.5  # [ft/s]
    c = c_f_m(aircraft, x, u)
    p_s = (c[0]) * v * 60 / w  # [ft/min]
    return p_s


def static_margin(plane, mach):
    """return longitudinal static margin."""
    c_m_a = Aircraft(plane, mach).c_m_alpha()
    c_l_a = Aircraft(plane, mach).c_l_alpha()
    sm = -c_m_a / c_l_a * 100
    return sm


def takeoff_ground_roll(aircraft, x_0, u_0):
    """return derivatives for aircraft on ground."""
    m = aircraft['weight']['weight']/g
    j = aircraft['weight']['inertia']
    c = c_f_m(aircraft, x_0, u_0)
    c_t, c_g, normal_loads = landing_gear_loads(aircraft, x_0, c, True)
    dxdt = nonlinear_eom(x_0, m, j, c_t)
    return dxdt


def rejected_takeoff(aircraft, s_f, x_0, u_0):
    """return derivatives for aircraft on ground."""
    v = [5 * x for x in range(1, 100)]
    m = aircraft['weight']['weight']/g
    j = aircraft['weight']['inertia']
    x = []
    for vi in v:
        x_0 [0] = 0.707 * vi
        c = c_f_m(aircraft, x_0, u_0)
        c_t, c_g, normal_loads = landing_gear_loads(aircraft, x_0, c, True, brake=1)
        dxdt = nonlinear_eom(x_0, m, j, c_t)
        x.append(s_f + (vi ** 2) / (2 * dxdt[0]))
    return v, x