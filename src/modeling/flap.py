from numpy import array, concatenate, cross, deg2rad, tan
from src.modeling.LiftingSurface import LiftingSurface
from src.modeling.trapezoidal_wing import mac, root_chord, span, y_chord, y_mac


def c_f_delta_flap(wing, control, mach):
    """return flap force coefficient derivatives, empirical method."""
    s = wing['planform']
    w = LiftingSurface(wing)
    c_l_a = w.c_l_alpha_wing(mach)
    s_a = flap_area(wing, control)
    f = array([-0.0125 * control['cf_c'] * s_a / s, 0, -c_l_a * control['cf_c'] * s_a / s])
    return f


def c_f_m_flap(wing, control, mach, cg):
    """return flap force and moment coefficient derivatives, empirical method."""
    y_scalar = control['b_2'] / abs(control['b_2'])
    s = wing['planform']
    b = span(wing['aspect_ratio'], s)
    taper = wing['taper']
    c_r = root_chord(wing['aspect_ratio'], s, taper)
    if y_scalar > 0:
        y_w = b * control['b_2'] / 2
        x_w = abs(y_w) * tan(deg2rad(wing['sweep_LE']))
        c_r_f = y_chord(abs(control['b_1']) * b / 2, c_r, b, taper)
        c_t_f = y_chord(abs(control['b_2']) * b / 2, c_r, b, taper)
    else:
        y_w = b * control['b_1'] / 2
        x_w = abs(y_w) * tan(deg2rad(wing['sweep_LE']))
        c_r_f = y_chord(abs(control['b_2']) * b / 2, c_r, b, taper)
        c_t_f = y_chord(abs(control['b_1']) * b / 2, c_r, b, taper)
    s_f = flap_area(wing, control)
    b_f = flap_span(wing, control)
    ar_f = (b_f ** 2) / s_f
    taper_f = c_t_f / c_r_f
    y_f = y_mac(ar_f, s_f, taper_f)
    cbar_f = mac(ar_f, s_f, taper_f)
    ac_f = array([y_f * tan(deg2rad(wing['sweep_LE'])) + cbar_f / 4, y_f * y_scalar, 0])
    ac = (ac_f + array([x_w, y_w, 0]) + array([wing['station'], wing['buttline'], wing['waterline']]))
    r = (ac - cg) * array([-1, 1, -1])
    f = c_f_delta_flap(wing, control, mach)
    m = cross(r, f)
    c = concatenate((f, m))
    return c


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
