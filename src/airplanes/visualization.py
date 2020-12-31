from matplotlib import pyplot as plt
from src.modeling.trapezoidal_wing import span, root_chord
from numpy import array, tan, deg2rad, sqrt, concatenate, zeros


def print_plane(plane):
    x_w, y_w, z_w = print_wing(plane['wing'])
    x_h, y_h, z_h = print_wing(plane['horizontal'])
    x_n, y_n, z_n = print_nose_gear(plane['landing_gear'])
    x_m, y_m_1, y_m_2, z_m = print_main_gear(plane['landing_gear'])
    x_v, y_v, z_v = print_vt(plane['vertical'])
    x_f, y_f, z_f = print_fuselage(plane['fuselage'])

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x_w, y_w)
    ax.plot(x_v, y_v)
    ax.plot(x_h, y_h)
    ax.plot(x_n, y_n)
    ax.plot(x_m, y_m_1)
    ax.plot(x_m, y_m_2)
    ax.plot(x_f, y_f)
    for ie in range(1, plane['propulsion']['n_engines']+1):
        x_e, y_e, z_e = print_prop(plane['propulsion']["engine_%d" % ie])
        ax.plot(x_e, y_e)
    ax.set_aspect(aspect=1)
    plt.show()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(y_w, z_w)
    ax.plot(y_v, z_v)
    ax.plot(y_h, z_h)
    ax.plot(y_n, z_n)
    ax.plot(y_m_1, z_m)
    ax.plot(y_m_2, z_m)
    ax.plot(y_f, z_f)
    for ie in range(1, plane['propulsion']['n_engines']+1):
        x_e, y_e, z_e = print_prop(plane['propulsion']["engine_%d" % ie])
        ax.plot(y_e, z_e)
    ax.set_aspect(aspect=1)
    plt.show()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x_w, z_w)
    ax.plot(x_v, z_v)
    ax.plot(x_h, z_h)
    ax.plot(x_n, z_n)
    ax.plot(x_m, z_m)
    ax.plot(x_f, z_f)
    for ie in range(1, plane['propulsion']['n_engines']+1):
        x_e, y_e, z_e = print_prop(plane['propulsion']["engine_%d" % ie])
        ax.plot(x_e, z_e)
    ax.set_aspect(aspect=1)
    plt.show()


def print_wing(wing):
    b = span(wing['aspect_ratio'], wing['planform'])
    c_r = root_chord(wing['aspect_ratio'], wing['planform'], wing['taper'])
    c_t = c_r * wing['taper']
    x = wing['station'] + array([0, b/2*tan(deg2rad(wing['sweep_LE'])), b/2*tan(deg2rad(wing['sweep_LE'])) + c_t,
                                 c_r, b/2*tan(deg2rad(wing['sweep_LE'])) + c_t, b/2*tan(deg2rad(wing['sweep_LE'])), 0])
    y = array([0, b/2, b/2, 0, -b/2, -b/2, 0])
    z = wing['waterline'] + array([0, b/2*tan(deg2rad(wing['dihedral'])), b/2*tan(deg2rad(wing['dihedral'])), 0,
                                   b / 2 * tan(deg2rad(wing['dihedral'])), b/2*tan(deg2rad(wing['dihedral'])), 0])
    return x, y, z


def print_vt(wing):
    b = span(wing['aspect_ratio'], wing['planform'], mirror=0)
    c_r = root_chord(wing['aspect_ratio'], wing['planform'], wing['taper'])
    c_t = c_r * wing['taper']
    x = wing['station'] + array([0, b/2*tan(deg2rad(wing['sweep_LE'])), b/2*tan(deg2rad(wing['sweep_LE'])) + c_t,
                                 c_r, 0])
    z = array([0, b, b, 0, 0]) + wing['waterline']
    y = array([0, 0, 0, 0, 0])
    return x, y, z


def print_nose_gear(gear):
    x, y, z = print_tire(gear['nose_diameter'])
    x = gear['nose'][0] + x
    y = gear['nose'][1] + y
    z = gear['nose'][2] + z
    return x, y, z


def print_main_gear(gear):
    x, y, z = print_tire(gear['main_diameter'])
    x = gear['main'][0] + x
    y_1 = gear['main'][1] + y
    y_2 = -gear['main'][1] + y
    z = gear['main'][2] + z
    return x, y_1, y_2, z


def print_prop(engine):
    y, x, z = print_tire(engine['diameter'])
    x = engine['station'] + x
    y = engine['buttline'] + y
    z = engine['waterline'] + z
    return x, y, z


def print_tire(r):
    x_i = array([x * r / 20 for x in range(0, 21)])
    y_i = sqrt(r**2 - x_i**2)
    x_rev = -x_i[::-1]
    y_rev = sqrt(r**2 - x_rev**2)
    x = concatenate((x_rev[0:-2], x_i))
    y = concatenate((y_rev[0:-2], y_i))
    x = concatenate((x, x[::-1]))
    z = concatenate((y, -y[::-1]))
    y = zeros(len(x))
    return x, y, z


def print_fuselage(fuselage):
    theta_cockpit = 45
    theta_tailstrike = 12
    l_tail = fuselage['length'] - fuselage['l_cabin'] - fuselage['l_cockpit']
    x = array([0, fuselage['l_cockpit'], fuselage['l_cockpit'] + fuselage['l_cabin'], fuselage['length'],
               fuselage['l_cockpit'] + fuselage['l_cabin'], fuselage['l_cockpit'], 0])
    y = array([0, 0, 0, 0, 0, 0, 0])
    z = array([fuselage['height'] - fuselage['l_cockpit'] * tan(deg2rad(theta_cockpit)),
               fuselage['height'],
               fuselage['height'],
               l_tail * tan(deg2rad(theta_tailstrike)),
               0, 0, fuselage['height'] - fuselage['l_cockpit'] * tan(deg2rad(theta_cockpit))])
    return x, y, z
