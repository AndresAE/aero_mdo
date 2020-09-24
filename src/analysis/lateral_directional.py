from control import damp, StateSpace
from matplotlib import pyplot as plt
from numpy import arctan, array, interp, log, max, min, unique, pi, real, sqrt, zeros
from src.common import Atmosphere, Earth
from src.common.equations_of_motion import nonlinear_eom_to_ss
from src.modeling.Aircraft import Aircraft
g = Earth(0).gravity()  # f/s2


def directional_stability(plane, mach, alpha):
    c_n_b = Aircraft(plane, mach).c_n_beta(alpha)
    return c_n_b


def lateral_stability(plane, mach, alpha):
    c_r_b = Aircraft(plane, mach).c_r_beta(alpha)
    return c_r_b


def dutch_roll_mode(aircraft, x_0, u_0):
    """return dutch-roll modal parameters."""
    wn_dr, zeta_dr, t_r, t_s = latdir_modes(aircraft, x_0, u_0)
    return wn_dr, zeta_dr


def roll_mode(aircraft, x_0, u_0):
    """return roll modal parameters."""
    wn_dr, zeta_dr, t_r, t_s = latdir_modes(aircraft, x_0, u_0)
    return t_r


def spiral_mode(aircraft, x_0, u_0):
    """return dutch-roll modal parameters."""
    wn_dr, zeta_dr, t_r, t_s = latdir_modes(aircraft, x_0, u_0)
    t_2_d = t_s * log(2)
    return t_2_d


def latdir_modes(aircraft, x_0, u_0):
    """calculate lateral-directional modal parameters."""
    j = aircraft['weight']['inertia']
    m = aircraft['weight']['weight'] / g  # slug
    # x = [u v w phi theta psi p q r p_n p_e h]
    x_ss = [1, 3, 6, 8]
    a, b, c, d = nonlinear_eom_to_ss(aircraft, x_ss, [0, 2], x_0, u_0, m, j)
    sys = StateSpace(a, b, c, d)
    wn, zeta, poles = damp(sys)
    wn_dr = unique(max(wn[abs(zeta) != 1]))  # [rad/s]
    zeta_dr = unique(max(zeta[wn == wn_dr]))  # []
    re = real(poles)
    t = unique(re[abs(zeta) == 1])
    if len(t) == 2:
        t_r = -1 / min(t)
        t_s = -1 / max(t)
    else:
        t_r = float("nan")
        t_s = float("nan")
    return wn_dr, zeta_dr, t_r, t_s


def plot_dr():
    """dutch roll patches."""
    x_1 = [0, 20]
    y_1 = [0.08, 0.08]
    x_2 = [0.4, 0.4]
    y_2 = [0, 20]
    plt.plot(x_1, y_1, 'k')
    plt.plot(x_2, y_2, 'k')
    plt.xlim((0, 10))
    plt.ylim((0, 1))
    plt.ylabel('Dutch-Roll Damping Ratio')
    plt.xlabel('Dutch-Roll Natural Frequency [rad/s]')


# def crosswind(aircraft, x, xwind_kts):
#     """rudder and aileron required to fly in crosswind."""
#     xwind_fps = xwind_kts * nautical_miles_to_ft() / 3600  # [ft/s]
#     a = Atmosphere(x[-1]).speed_of_sound()  # [ft/s]
#     v = sqrt(x[0]**2 + x[1]**2 + x[2]**2)  # [ft/s]
#     mach = v / a  # []
#     alpha = arctan(x[2]/v)  # [rad]
#     beta = arctan((x[1]+xwind_fps)/v)  # [rad]
#     dr, da = trim_beta(aircraft, mach, rad2deg(alpha), rad2deg(beta))  # [deg]
#     return dr, da


# def time_to_bank(aircraft, x, de, throttle, t_req, phi_req):
#     """find aileron required to bank in t_req to phi_req."""
#     dt = 0.01  # [sec]
#     u = array([[0, 0, 1, 1], [float(de), float(de), float(de), float(de)], [0, 0, 0, 0]])
#     t_i = [0, 0.99, 1, t_req + 1]
#     d_ail = array([0.05, 0.1, 0.3, 0.5, 0.7, 0.9, 1.1])
#     phi = zeros(len(d_ail))
#     for ii in range(0, len(d_ail)):
#         u[0, :] = [0, 0, d_ail[ii], d_ail[ii]]
#         x_out, a_p, u_out, t_out = nonlinear_6dof_sim(aircraft, x, u, throttle, t_i, dt)
#         phi[ii] = x_out[-1, 3] * 180 / pi
#     ail_req = interp(phi_req, phi, d_ail) * 180 / pi
#     return ail_req
