"""Contains aerodynamic calculations."""
from numpy import log10, pi, sqrt, tan
from src.common import Atmosphere
from src.modeling.trapezoidal_wing import sweep_x


def dynamic_pressure(mach, altitude):
    """returns incompressible dynamic pressure."""
    rho = Atmosphere(altitude).air_density()
    a = Atmosphere(altitude).speed_of_sound()
    v = mach * a
    q_bar = 0.5 * rho * v ** 2
    return q_bar


def friction_coefficient(mach, altitude, x_ref):
    """return air friction coefficient for flight condition."""
    re = reynolds_number(mach, altitude, x_ref)  # []
    if re < 500000:
        c_f = 1.328 / sqrt(re)  # []
    else:
        c_f = 0.455 / (log10(re)) ** 2.58  # []
    return c_f


def polhamus(c_l_alpha, ar, mach, taper, sweep_le):
    """returns lift curve slope using Polhamus method."""
    if ar < 4:
        k = 1 + ar * (1.87 - 0.000233 * sweep_le * pi / 180) / 100
    else:
        k = 1 + ((8.2 - 2.3 * sweep_le * pi / 180) - ar * (0.22 - 0.153 * sweep_le * pi / 180)) / 100

    sweep_2 = sweep_x(ar, taper, sweep_le, 0.5)

    beta = sqrt(1 - mach ** 2)
    root = 4 + (((ar ** 2) * (beta ** 2)) / (k ** 2) * (1 + (tan(sweep_2 * pi / 180) ** 2) / (beta ** 2)))
    c_l_alpha_wing = c_l_alpha * ar / (2 + sqrt(root))  # [1/rad]

    return c_l_alpha_wing


def pressure_drag(t_c):
    """return pressure drag coefficient factor."""
    c_d_p = 1 + 1.5 * (t_c ** (3/2)) + 7 * (t_c ** 3)
    return c_d_p


def reynolds_number(mach, altitude, x_ref):
    """return reynolds number."""
    rho = Atmosphere(altitude).air_density()  # [slug/ft^3]
    mu = Atmosphere(altitude).viscosity()
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    v = mach * a  # [ft/s]
    re = rho * v * x_ref / mu  # []
    return re
