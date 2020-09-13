"""Contains aerodynamic calculations."""
from numpy import log10, pi, sqrt, tan
from common.atmosphere import air_density, speed_of_sound, viscosity
from modeling.trapezoidal_wing import sweep_x


def polhamus(c_l_alpha, ar, mach, taper, sweep_le):
    """returns lift curve slope using Polhamus method."""
    if ar < 4:
        k = 1 + ar * (1.87 - 0.000233 * sweep_le * pi / 180) / 100
    else:
        k = 1 + ((8.2 - 2.3 * sweep_le * pi / 180) - ar * (0.22 - 0.153 * sweep_le)) / 100

    sweep_2 = sweep_x(ar, taper, sweep_le, 0.5)

    beta = sqrt(1 - mach ** 2)
    root = 4 + ((ar ** 2 * beta ** 2) / (k ** 2) * (1 + (tan(sweep_2 * pi / 180) ** 2) / (beta ** 2)))
    c_l_alpha_wing = c_l_alpha * ar / (2 + sqrt(root))  # [1/rad]

    return c_l_alpha_wing


def friction_coefficient(mach, altitude, x_ref):
    """return air friction coefficient for flight condition."""
    re = reynolds_number(mach, altitude, x_ref)  # []
    if re < 500000:
        c_f = 1.328 / sqrt(re)  # []
    else:
        c_f = 0.455 / (log10(re)) ** 2.58  # []
    return c_f


def reynolds_number(mach, altitude, x_ref):
    """return reynolds number."""
    rho = air_density(altitude)  # [slug/ft^3]
    mu = viscosity(altitude)
    a = speed_of_sound(altitude)  # [ft/s]
    v = mach * a  # [ft/s]
    re = rho * v * x_ref / mu  # []
    return re


def dynamic_pressure(mach, altitude):
    """returns incompressible dynamic pressure."""
    rho = air_density(altitude)
    a = speed_of_sound(altitude)
    v = mach * a
    q_bar = 0.5 * rho * v ** 2
    return q_bar
