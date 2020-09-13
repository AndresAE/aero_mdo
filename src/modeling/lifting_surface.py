"""Returns force and moment coefficients for lifting surfaces."""
from modeling.aerodynamics import polhamus, friction_coefficient
from modeling.trapezoidal_wing import root_chord, mac, span, sweep_x, x_mac, y_mac
from numpy import sqrt, cos, deg2rad


def c_l_alpha_wing(wing, mach):
    """return lifting surface lift curve slope."""
    c_l_alpha = wing['c_l_alpha']  # [1/rad] 2-D
    ar = wing['aspect_ratio']  # []
    sweep_le = wing['sweep_LE']  # [deg]
    taper = wing['taper']  # []
    c_l_alpha_3d = polhamus(c_l_alpha, ar, mach, taper, sweep_le)   # [1/rad]
    return c_l_alpha_3d


def d_epsilon_d_alpha(wing, ht, mach):
    """return downwash gradient wrt angle of attack."""
    ar = wing['aspect_ratio']  # []
    taper = wing['taper']  # []
    root_chord_wing = root_chord(ar, wing['planform'], taper)  # [ft]
    root_chord_ht = root_chord(ht['aspect_ratio'], ht['planform'], ht['taper'])  # [ft]
    x_wh = (ht['station'] + root_chord_ht / 4) - (wing['station'] + root_chord_wing / 4)  # [ft]
    z_wh = ht['waterline'] - ht['waterline']  # [ft]
    b = span(ar, wing['planform'])  # [ft]
    r = 2 * x_wh / b  # []
    m = 2 * z_wh / b  # []
    k_ar = (1 / ar) - 1 / (1 + ar ** 1.7)  # []
    k_taper = (10 - 3 * taper) / 7  # []
    k_mr = (1 - (m / 2)) / (r ** 0.333)  # []
    sweep_25 = sweep_x(ar, taper, wing['sweep_LE'], 0.25)  # [deg]
    beta = sqrt(1 - mach ** 2)  # []
    de_da = 4.44 * beta * (k_ar * k_taper * k_mr * sqrt(cos(deg2rad(sweep_25)))) ** 1.19  # []
    return de_da


def aerodynamic_center(wing):
    """return lifting surface aerodynamic center."""
    c_bar = mac(wing['aspect_ratio'], wing['planform'], wing['taper'])  # [ft]
    y = y_mac(wing['aspect_ratio'], wing['planform'], wing['taper'])  # [ft]
    x = x_mac(y, wing['sweep_LE'])  # [ft]
    x_ac = wing['station'] + x + c_bar / 4  # [ft]
    return x_ac


def parasite_drag(wing, mach, altitude):
    """return parasitic drag coefficient of the lifting surface."""
    s_wet_s = 2 + 2 * (wing['airfoil_thickness'] / wing['aspect_ratio']) + 2 * wing['airfoil_thickness']  # []
    c_bar = mac(wing['aspect_ratio'], wing['planform'], wing['taper'])  # [ft]
    c_f = friction_coefficient(mach, altitude, c_bar)  # []
    c_d_0 = 1.25 * c_f * s_wet_s  # []
    return c_d_0


def d_sigma_d_beta(aircraft):
    """return sidewash gradient wrt sideslip."""
    wing = aircraft['wing']
    vt = aircraft['vertical']
    fuselage = aircraft['fuselage']
    ar = wing['aspect_ratio']
    s_w = wing['planform']
    s_v = vt['planform']
    z_w = wing['waterline']
    d = fuselage['width']
    sweep_4 = sweep_x(ar, wing['taper'], wing['sweep_LE'], 0.25)
    eta_ds_db = 0.724 + 3.06 * (s_v / s_w) / (1 + cos(deg2rad(sweep_4))) + 0.4 * z_w / d + 0.009 * ar
    return eta_ds_db
