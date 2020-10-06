from numpy import cos, deg2rad, mean, sin, sqrt
from src.common import Atmosphere
from src.common.Earth import Earth
from src.modeling.Aircraft import Aircraft


def master_constraint(aircraft, wing_loading, mach, altitude, n, gamma, a_x):
    """master constraint equation for flight."""
    g = Earth(0).gravity()
    rho = Atmosphere(altitude).air_density()
    rho_sl = Atmosphere(0).air_density()
    a = Atmosphere(altitude).speed_of_sound()
    v = a*mach
    q_bar = 0.5*rho*v**2

    c_d_0 = Aircraft(aircraft, mach).c_d_zero(altitude)
    c_l_a = Aircraft(aircraft, mach).c_l_alpha()

    t_w = (q_bar*c_d_0/wing_loading
           + wing_loading*(n**2)*(cos(deg2rad(gamma))**2)/(q_bar*c_l_a)
           + n*sin(deg2rad(gamma)) + a_x/g)*rho_sl/rho
    return t_w


def stall_speed(mach, altitude, c_l_max, n, gamma):
    """stall speed constraint equation."""
    rho = Atmosphere(altitude).air_density()
    a = Atmosphere(altitude).speed_of_sound()
    v = a * mach
    q_bar = 0.5 * rho * v ** 2
    w_s = q_bar*c_l_max/(n*cos(deg2rad(gamma)))
    return w_s


def takeoff(aircraft, wing_loading, s_to, altitude, mu):
    """takeoff constraint equation."""
    g = Earth(0).gravity()
    rho = Atmosphere(altitude).air_density()
    rho_sl = Atmosphere(0).air_density()

    c_l_max = 1
    a = Atmosphere(altitude).speed_of_sound()
    q_stall = wing_loading / c_l_max
    q_v_avg = 0.5 * q_stall
    mach_avg = sqrt(q_v_avg/(0.5*rho))/a

    c_d_0 = Aircraft(aircraft, mean(mach_avg)).c_d_zero(altitude)
    c_l_a = Aircraft(aircraft, mean(mach_avg)).c_l_alpha()
    c_l_0 = Aircraft(aircraft, mean(mach_avg)).c_l_zero()

    d_w = q_v_avg*(c_d_0+(c_l_0**2)/c_l_a)/wing_loading
    l_w = q_v_avg*c_l_0/wing_loading

    t_w = (1.44*wing_loading/(rho*c_l_max*s_to*g) + d_w + mu*(1-l_w))*rho_sl/rho
    return t_w
