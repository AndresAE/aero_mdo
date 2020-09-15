from numpy import array, concatenate, cross, deg2rad, tan
from src.modeling import LiftingSurface
from src.modeling.trapezoidal_wing import mac, root_chord, span, y_chord, y_mac


def flap_area(wing, control):
    """return flapped area."""
    s = wing['planform']
    b = span(wing['aspect_ratio'], s)
    taper = wing['taper']
    c_r = root_chord(wing['aspect_ratio'], s, taper)
    s_a = ((control['b_2'] - control['b_1']) * b / 4 *
           (y_chord(abs(control['b_2']) * b / 2, c_r, b, taper) + y_chord(abs(control['b_1']) * b / 2, c_r, b, taper)))
    return s_a


def flap_span(wing, control):
    """return flap span."""
    b = span(wing['aspect_ratio'], wing['planform'])
    b_f = (control['b_2'] - control['b_1']) * b / 2
    return b_f


def c_f_delta_flap(wing, control, mach):
    """return flap force coefficient derivatives."""
    s = wing['planform']
    w = LiftingSurface(wing)
    c_l_a = w.c_l_alpha_wing(mach)
    s_a = flap_span(wing, control)
    f = array([0, 0, c_l_a * control['cf_c'] * s_a / s])
    return f


def c_f_m_flap(wing, control, mach, cg):
    """return flap force and moment coefficient derivatives."""
    y_scalar = control['b_2'] / abs(control['b_2'])
    f = c_f_delta_flap(wing, control, mach)
    s = wing['planform']
    b = span(wing['aspect_ratio'], s)
    taper = wing['taper']
    cbar = mac(wing['aspect_ratio'], wing['planform'], wing['taper'])
    y_w = y_mac(wing['aspect_ratio'], wing['planform'], wing['taper'])
    x_w = y_w * tan(deg2rad(wing['sweep_LE']))
    z_w = y_w * tan(deg2rad(wing['dihedral']))
    c_r = root_chord(wing['aspect_ratio'], s, taper)
    c_t_f = y_chord(control['b_2'], c_r, b, taper)
    c_r_f = y_chord(control['b_1'], c_r, b, taper)
    s_f = flap_area(wing, control)
    b_f = flap_span(wing, control)
    ar_f = (b_f ** 2) / s_f
    taper_f = c_t_f / c_r_f
    y_f = y_mac(ar_f, s_f, taper_f)
    cbar_f = mac(ar_f, s_f, taper_f)
    ac_f = (array([y_f * tan(deg2rad(wing['sweep_LE'])) + cbar_f / 4, y_f, y_f * tan(deg2rad(wing['dihedral']))]) +
            array([x_w, y_w, z_w]) +
            array([wing['station'], wing['buttline'], wing['waterline']]))
    r = -(ac_f * [1, y_scalar, 1] - cg)
    m = cross(r, f) / array([b, cbar, b])
    c = concatenate((f, m))
    return c
