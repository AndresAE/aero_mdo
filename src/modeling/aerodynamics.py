"""Contains aerodynamic calculations."""
import avlwrapper as avl
from numpy import array, concatenate, deg2rad, linspace, log10, pi, sort, sqrt, tan, unique, zeros
from src.common import Atmosphere
from src.common.report_tools import save_aero_model
from src.modeling.trapezoidal_wing import mac, root_chord, span, sweep_x, y_chord


def create_aero_model_avl(aircraft, requirements):
    """create aero model using aircraft requirements with linear AVL method."""
    # baseline_sweep
    mach = linspace(requirements['flight_envelope']['mach'][0], requirements['flight_envelope']['mach'][1], num=4)
    alpha = linspace(requirements['flight_envelope']['alpha'][0], requirements['flight_envelope']['alpha'][1], num=5)
    alpha = sort(unique(concatenate((alpha, array([0])))))
    beta = linspace(requirements['flight_envelope']['beta'][0], requirements['flight_envelope']['beta'][1], num=5)
    p = linspace(requirements['flight_envelope']['p'][0], requirements['flight_envelope']['p'][1], num=5)
    q = linspace(requirements['flight_envelope']['q'][0], requirements['flight_envelope']['q'][1], num=5)
    r = linspace(requirements['flight_envelope']['r'][0], requirements['flight_envelope']['r'][1], num=5)
    d_a = linspace(aircraft['wing']['control_4']['limits'][0], aircraft['wing']['control_4']['limits'][1], num=5)
    d_e = linspace(aircraft['horizontal']['control_2']['limits'][0],
                   aircraft['horizontal']['control_2']['limits'][1], num=4)
    d_e = sort(unique(concatenate((d_e, array([0])))))
    d_r = linspace(aircraft['vertical']['control_1']['limits'][0],
                   aircraft['vertical']['control_1']['limits'][1], num=5)

    model = {'mrc': aircraft['weight']['cg'],
             'baseline': {'mach': mach, 'alpha': alpha,
                          'cfm': sweep_avl_2d(aircraft, mach, alpha,
                                              'run_avl(aircraft, ix, iy, 0, 0, 0, 0, [0, 0, 0, 0])')},
             'lat_dir': {'mach': mach, 'beta': beta,
                         'cfm': sweep_avl_2d(aircraft, mach, beta,
                                             'run_avl(aircraft, ix, 0, iy, 0, 0, 0, [0, 0, 0, 0])')},
             'aileron': {'mach': mach, 'd_aileron': d_a,
                         'cfm': sweep_avl_2d(aircraft, mach, d_a,
                                             'run_avl(aircraft, ix, 0, 0, 0, 0, 0, [iy, 0, 0, 0])')},
             'elevator': {'mach': mach, 'd_elevator': d_e,
                          'cfm': sweep_avl_2d(aircraft, mach, d_e,
                                              'run_avl(aircraft, ix, 0, 0, 0, 0, 0, [0, iy, 0, 0])')},
             'rudder': {'mach': mach, 'd_rudder': d_r, 'alpha': alpha,
                        'cfm': sweep_avl_2d(aircraft, mach, d_r,
                                            'run_avl(aircraft, ix, 0, 0, 0, 0, 0, [0, 0, iy, 0])')},
             'p': {'mach': mach, 'p': p,
                   'cfm': sweep_avl_2d(aircraft, mach, p, 'run_avl(aircraft, ix, 0, 0, iy, 0, 0, [0, 0, 0, 0])')},
             'q': {'mach': mach, 'q': q,
                   'cfm': sweep_avl_2d(aircraft, mach, q, 'run_avl(aircraft, ix, 0, 0, 0, iy, 0, [0, 0, 0, 0])')},
             'r': {'mach': mach, 'r': r,
                   'cfm': sweep_avl_2d(aircraft, mach, r, 'run_avl(aircraft, ix, 0, 0, 0, 0, iy, [0, 0, 0, 0])')},
             }
    save_aero_model(model, aircraft['name'])  # create directory if it doesn't exist
    return model


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


def run_avl(aircraft, mach, alpha, beta, p, q, r, u, iplot=0):
    """run Athena Vortex Lattice Method."""
    case_name = 'zero_alpha'
    roll_rate = deg2rad(p)  # [rad/s]
    pitch_rate = deg2rad(q)  # [rad/s]
    yaw_rate = deg2rad(r)  # [rad/s]
    d_aileron = u[0]  # deg
    d_elevator = u[1]  # deg
    d_rudder = u[2]  # deg
    gains = [-1, 1, 1]

    wing = aircraft['wing']
    ht = aircraft['horizontal']
    vt = aircraft['vertical']
    wing_aspect_ratio = wing['aspect_ratio']
    wing_area = wing['planform']
    wing_taper = wing['taper']
    wing_root_le_pnt = avl.Point(wing['station'], wing['buttline'], wing['waterline'])
    ht_aspect_ratio = ht['aspect_ratio']
    ht_area = ht['planform']
    ht_root_le_pnt = avl.Point(ht['station'], ht['buttline'], ht['waterline'])
    vt_aspect_ratio = vt['aspect_ratio']
    vt_area = vt['planform']
    vt_root_le_pnt = avl.Point(vt['station'], vt['buttline'], vt['waterline'])

    a = Atmosphere(0).speed_of_sound()
    ref_pnt = avl.Point(aircraft['weight']['cg'][0], aircraft['weight']['cg'][1], aircraft['weight']['cg'][2])

    # Wing -------------------------------------------------------------------------------------------------------------
    wing_span = span(wing_aspect_ratio, wing_area)
    wing_mac = mac(wing_aspect_ratio, wing_area, wing_taper)
    sections = list()
    if not wing['control_4']['b_1'] == 0:
        sections.append(avl_section(wing_root_le_pnt[1], 0, wing, 1, []))
    sections.append(avl_section(wing_root_le_pnt[1] + wing_span * 0.5 * wing['control_4']['b_1'],
                    wing['control_4'], wing, 1, 'aileron', duplicate_sign=-1))
    sections.append(avl_section(wing_root_le_pnt[1] + wing_span * 0.5 * wing['control_4']['b_2'],
                    wing['control_4'], wing, 1, 'aileron', duplicate_sign=-1))
    if not wing['control_4']['b_2'] == 1:
        sections.append(avl_section(wing_root_le_pnt[1] + wing_span * 0.5, 0, wing, 1, []))

    # y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
    wing = avl.Surface(name='wing',
                       n_chordwise=12,
                       chord_spacing=avl.Spacing.cosine,
                       n_spanwise=20,
                       span_spacing=avl.Spacing.cosine,
                       y_duplicate=0.0,
                       sections=sections)

    # HT ---------------------------------------------------------------------------------------------------------------
    ht_span = span(ht_aspect_ratio, ht_area)
    sections = list()
    if not ht['control_2']['b_1'] == 0:
        sections.append(avl_section(ht_root_le_pnt[1], 0, ht, 1, []))
    sections.append(avl_section(ht_root_le_pnt[1] + ht_span * 0.5 * ht['control_2']['b_1'],
                    ht['control_2'], ht, 1, 'elevator'))
    sections.append(avl_section(ht_root_le_pnt[1] + ht_span * 0.5 * ht['control_2']['b_2'],
                    ht['control_2'], ht, 1, 'elevator'))
    if not ht['control_2']['b_2'] == 1:
        sections.append(avl_section(ht_root_le_pnt[1] + ht_span * 0.5, 0, ht, 1, []))
    horizontal_tail = avl.Surface(name='horizontal_tail',
                                  n_chordwise=12,
                                  chord_spacing=avl.Spacing.cosine,
                                  n_spanwise=20,
                                  span_spacing=avl.Spacing.cosine,
                                  y_duplicate=0.0,
                                  sections=sections)
    # VT ---------------------------------------------------------------------------------------------------------------
    vt_span = span(vt_aspect_ratio, vt_area, mirror=0)
    sections = list()
    if not vt['control_1']['b_1'] == 0:
        sections.append(avl_section(vt_root_le_pnt[1], 0, vt, 0, []))
    sections.append(avl_section(vt_root_le_pnt[1] + vt_span * vt['control_1']['b_1'],
                    vt['control_1'], vt, 0, 'rudder'))
    sections.append(avl_section(vt_root_le_pnt[1] + vt_span * vt['control_1']['b_2'],
                    vt['control_1'], vt, 0, 'rudder'))
    if not vt['control_1']['b_2'] == 1:
        sections.append(avl_section(vt_root_le_pnt[1] + vt_span, 0, vt, 0, []))
    vertical_tail = avl.Surface(name='vertical_tail',
                                n_chordwise=12,
                                chord_spacing=avl.Spacing.cosine,
                                n_spanwise=20,
                                span_spacing=avl.Spacing.cosine,
                                sections=sections)
    # Setup ------------------------------------------------------------------------------------------------------------
    aircraft = avl.Geometry(name='aircraft',
                            reference_area=wing_area,
                            reference_chord=wing_mac,
                            reference_span=wing_span,
                            reference_point=ref_pnt,
                            mach=mach,
                            surfaces=[wing, horizontal_tail, vertical_tail])

    def show_treffz(session_1):
        if 'gs_bin' in session_1.config.settings:
            images = session_1.save_trefftz_plots()
            for iimg in images:
                avl.show_image(iimg)
        else:
            for idx, _ in enumerate(session_1.cases):
                session_1.show_trefftz_plot(idx + 1)  # cases start from 1

    simple_case = avl.Case(name=case_name,
                           alpha=alpha, beta=beta,
                           aileron=gains[0] * d_aileron, elevator=gains[1] * d_elevator, rudder=gains[2] * d_rudder,
                           roll_rate=roll_rate * wing_span/(2 * mach * a),
                           pitch_rate=pitch_rate * wing_span/(2 * mach * a),
                           yaw_rate=yaw_rate * wing_span/(2 * mach * a))
    session = avl.Session(geometry=aircraft, cases=[simple_case])

    if iplot:
        if 'gs_bin' in session.config.settings:
            img = session.save_geometry_plot()[0]
            avl.show_image(img)
        else:
            session.show_geometry()

        show_treffz(session)

    # results are in a dictionary
    result = session.run_all_cases()
    cd = result[case_name]['Totals']['CXtot']
    cy = result[case_name]['Totals']['CYtot']
    cl = result[case_name]['Totals']['CLtot']
    cmr = result[case_name]['Totals']['Cltot']
    cmp = result[case_name]['Totals']['Cmtot']
    cmy = result[case_name]['Totals']['Cntot']
    print("cfm= {}".format([cd, cy, cl, cmr, cmp, cmy]))
    return [cd, cy, cl, cmr, cmp, cmy]


def avl_section(y, cs, wing, mirror, cs_name, duplicate_sign=1):
    """create avl wing section."""
    b = span(wing['aspect_ratio'], wing['planform'], mirror=mirror)
    c_r = root_chord(wing['aspect_ratio'], wing['planform'], wing['taper'], mirror=mirror)
    wing_root_le_pnt = avl.Point(wing['station'], wing['buttline'], wing['waterline'])
    if mirror:
        le_point = avl.Point(x=wing_root_le_pnt.x + y * tan(deg2rad(wing['sweep_LE'])),
                             y=y,
                             z=wing_root_le_pnt.z + y * tan(deg2rad(wing['dihedral'])))
        chord = y_chord(y, c_r, b, wing['taper'])
    else:
        le_point = avl.Point(x=wing_root_le_pnt.x + y * tan(deg2rad(wing['sweep_LE'])),
                             y=wing_root_le_pnt.y,
                             z=wing_root_le_pnt.z + y)
        chord = y_chord(y, c_r, b * 2, wing['taper'])
    if cs == 0:
        section = avl.Section(leading_edge_point=le_point,
                              chord=chord,
                              airfoil=avl.NacaAirfoil(wing['airfoil']))
    else:
        control = avl.Control(name=cs_name,
                              gain=1,
                              x_hinge=1 - cs['cf_c'],
                              duplicate_sign=duplicate_sign)
        section = avl.Section(leading_edge_point=le_point,
                              chord=chord,
                              airfoil=avl.NacaAirfoil(wing['airfoil']),
                              controls=[control])
    return section


def sweep_avl_2d(aircraft, x, y, run_string):
    """execute sweep of key in AVL."""
    out = {
        'cd': zeros((len(x), len(y))),
        'cy': zeros((len(x), len(y))),
        'cl': zeros((len(x), len(y))),
        'cmr': zeros((len(x), len(y))),
        'cmp': zeros((len(x), len(y))),
        'cmy': zeros((len(x), len(y))),
    }
    ii = 0
    for ix in x:
        jj = 0
        for iy in y:
            out_avl = eval(run_string)
            out['cd'][ii, jj] = out_avl[0]
            out['cy'][ii, jj] = out_avl[1]
            out['cl'][ii, jj] = out_avl[2]
            out['cmr'][ii, jj] = out_avl[3]
            out['cmp'][ii, jj] = out_avl[4]
            out['cmy'][ii, jj] = out_avl[5]
            jj = jj + 1
        ii = ii + 1
    return out
