from common.unit_conversion import gravitational_acceleration
from modeling.aircraft import c_d_zero, c_l_alpha, c_l_zero
from common.atmosphere import air_density, speed_of_sound
from numpy import cos, deg2rad, mean, sin, sqrt


def master_constraint(aircraft, wing_loading, mach, altitude, n, gamma, a_x):
    """master constraint equation for flight."""
    g = gravitational_acceleration()
    rho = air_density(altitude)
    rho_sl = air_density(0)
    a = speed_of_sound(altitude)
    v = a*mach
    q_bar = 0.5*rho*v**2

    c_d_0 = c_d_zero(aircraft, mach, altitude)
    c_l_a = c_l_alpha(aircraft, mach)

    t_w = (q_bar*c_d_0/wing_loading
           + wing_loading*(n**2)*(cos(deg2rad(gamma))**2)/(q_bar*c_l_a)
           + n*sin(deg2rad(gamma)) + a_x/g)*rho_sl/rho
    return t_w


def stall_speed(mach, h, c_l_max, n, gamma):
    """stall speed constraint equation."""
    rho = air_density(h)
    a = speed_of_sound(h)
    v = a * mach
    q_bar = 0.5 * rho * v ** 2
    w_s = q_bar*c_l_max/(n*cos(deg2rad(gamma)))
    return w_s


def takeoff(aircraft, wing_loading, s_to, altitude, mu):
    """takeoff constraint equation."""
    g = gravitational_acceleration()
    rho = air_density(altitude)
    rho_sl = air_density(0)

    c_l_max = 1
    a = speed_of_sound(altitude)
    q_stall = wing_loading / c_l_max
    q_v_avg = 0.5 * q_stall
    mach_avg = sqrt(q_v_avg/(0.5*rho))/a

    c_d_0 = c_d_zero(aircraft, mean(mach_avg), altitude)
    c_l_0 = c_l_zero(aircraft, mean(mach_avg))
    c_l_a = c_l_alpha(aircraft, mean(mach_avg))

    d_w = q_v_avg*(c_d_0+(c_l_0**2)/c_l_a)/wing_loading
    l_w = q_v_avg*c_l_0/wing_loading

    t_w = (1.44*wing_loading/(rho*c_l_max*s_to*g) + d_w + mu*(1-l_w))*rho_sl/rho
    return t_w
