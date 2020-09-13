"""Contains trapezoidal wing calculations."""
from numpy import arctan, deg2rad, rad2deg, sqrt, tan


def mac(ar, s, taper):
    """returns mean aerodynamic chord length."""
    c_r = root_chord(ar, s, taper)  # [ft]
    mean_chord = 2 / 3 * c_r * (1 + taper + taper**2) / (1+taper)  # [ft]
    return mean_chord


def root_chord(ar, s, taper):
    """returns rectangular wing root chord."""
    b = span(ar, s)  # [ft]
    chord = 2 * s / (b * (1 + taper))  # [ft]
    return chord


def span(ar, s):
    """returns rectangular wing span."""
    b = sqrt(ar * s)  # [ft]
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


def y_mac(ar, s, taper):
    """return mean aerodynamic chord y buttline."""
    b = span(ar, s)  # [ft]
    y = b/6*(1+2*taper)/(1+taper)  # [ft]
    return y
