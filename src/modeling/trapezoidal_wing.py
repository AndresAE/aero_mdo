"""Contains trapezoidal wing calculations."""
from numpy import arctan, deg2rad, rad2deg, sqrt, tan


def mac(ar, s, taper, mirror=1):
    """returns mean aerodynamic chord length."""
    if mirror == 1:
        scalar = 1
    else:
        scalar = 2
    c_r = root_chord(ar, s * scalar, taper)  # [ft]
    mean_chord = 2 / 3 * c_r * (1 + taper + taper**2) / (1+taper)  # [ft]
    return mean_chord


def root_chord(ar, s, taper, mirror=1):
    """returns rectangular wing root chord."""
    if mirror == 1:
        scalar = 1
    else:
        scalar = 2
    b = span(ar, s * scalar)  # [ft]
    chord = 2 * s * scalar / (b * (1 + taper))  # [ft]
    return chord


def span(ar, s, mirror=1):
    """returns rectangular wing span."""
    if mirror == 1:
        scalar = 1
    else:
        scalar = 2
    b = sqrt(ar * s / scalar)  # [ft]
    return b


def sweep_x(ar, taper, sweep_le, x):
    """returns sweep at x/c ratio."""
    sweep_out = rad2deg(arctan(tan(deg2rad(sweep_le))-4*x*(1-taper)/(ar*(1+taper))))  # [deg]
    return sweep_out


def x_mac(y, sweep_le):
    """return leading edge coordinate wrt y."""
    x = y*tan(deg2rad(sweep_le))  # [ft]
    return x


def y_chord(y, c_r, b, taper):
    """return chord at location y for trapezoidal wing"""
    c = c_r * (2 * y * (taper - 1) / b + 1)
    return c


def y_mac(ar, s, taper, mirror=1):
    """return mean aerodynamic chord y buttline."""
    if mirror == 1:
        scalar = 1
    else:
        scalar = 2
    b = span(ar, s * scalar)  # [ft]
    y = b/6*(1+2*taper)/(1+taper)  # [ft]
    return y
