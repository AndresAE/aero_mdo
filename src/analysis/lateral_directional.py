from control import damp, StateSpace
from matplotlib import pyplot as plt
from numpy import array, gradient, linalg, log, max, mean, min, unique, real
from src.analysis.trim import trim_aileron_rudder_speed_nonlinear
from common import Gravity, Atmosphere
from src.analysis.controls_analysis import nonlinear_eom_to_ss
from common.rotations import body_to_wind
from src.modeling.Aircraft import Aircraft
from src.modeling.force_model import linear_aero, nonlinear_aero
g = Gravity(0).gravity()  # f/s2


def directional_stability(plane, mach, alpha):
    """return airplane static directional stability."""
    c_n_b = Aircraft(plane, mach).c_n_beta(alpha)
    return c_n_b


def dutch_roll_mode(aircraft, x_0, u_0):
    """return dutch-roll modal parameters."""
    wn_dr, zeta_dr, t_r, t_s = latdir_modes(aircraft, x_0, u_0)
    return wn_dr, zeta_dr


def lateral_stability(plane, mach, alpha):
    """return airplane static lateral stability."""
    c_r_b = Aircraft(plane, mach).c_r_beta(alpha)
    return c_r_b


def latdir_stability_nonlinear(plane, mach, altitude, alpha, de, delta=0.01):
    """return lateral directional stability derivatives using nonlinear model."""
    u = [0, de, 0, 0.01]
    betas = array([-delta, 0, delta])
    a = Atmosphere(altitude).speed_of_sound()
    v = a * mach
    cmr = []
    cmy = []
    for bi in betas:
        b2w = body_to_wind(alpha, bi)
        uvw = linalg.inv(b2w) @ array([v, 0, 0])
        x = [uvw[0], uvw[1], uvw[2], 0, alpha, 0, 0, 0, 0, 0, 0, altitude]
        if 'aero_model' in plane.keys():
            cfm = nonlinear_aero(plane, x, u)
        else:
            cfm = linear_aero(plane, x, u)
        cmr.append(cfm[3])
        cmy.append(cfm[5])
    cmr_b = mean(gradient(cmr, betas))
    cmy_b = mean(gradient(cmy, betas))
    return cmr_b, cmy_b


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


def minimum_control_speed_air(plane, altitude):
    """return minimum control speed in air for given altitude."""
    prop = plane['propulsion']
    c = []
    if prop['n_engines'] > 1:
        if prop['engine_1']['type'] == 'prop':
            pitch = prop['engine_1']['pitch']
            plane['propulsion']['engine_1']['pitch'] = 0
            c = trim_aileron_rudder_speed_nonlinear(plane, altitude, 0, 0, 0)
            plane['propulsion']['engine_1']['pitch'] = pitch
        elif prop['engine_1']['type'] == 'jet':
            thrust = prop['engine_1']['thrust']
            plane['propulsion']['engine_1']['thrust'] = 0
            c = trim_aileron_rudder_speed_nonlinear(plane, altitude, 0, 0, 0)
            plane['propulsion']['engine_1']['thrust'] = thrust
        vmca = c[4]
    else:
        vmca = 0
    return vmca


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


def roll_mode(aircraft, x_0, u_0):
    """return roll modal parameters."""
    wn_dr, zeta_dr, t_r, t_s = latdir_modes(aircraft, x_0, u_0)
    return t_r


def spiral_mode(aircraft, x_0, u_0):
    """return dutch-roll modal parameters."""
    wn_dr, zeta_dr, t_r, t_s = latdir_modes(aircraft, x_0, u_0)
    t_2_d = t_s * log(2)
    return t_2_d
