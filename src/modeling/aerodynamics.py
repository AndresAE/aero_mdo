"""Contains aerodynamic calculations."""
import avlwrapper as avl
from numpy import deg2rad, log10, pi, rad2deg, sqrt, tan
from src.common import Atmosphere
from src.modeling.trapezoidal_wing import mac, root_chord, span,sweep_x, x_mac


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


def run_avl(aircraft, mach, alpha, beta, p, q, r, u):
    case_name = 'zero_alpha'
    roll_rate = deg2rad(p)
    pitch_rate = deg2rad(q)
    yaw_rate = deg2rad(r)
    d_aileron = rad2deg(u[0])  # deg
    d_elevator = rad2deg(u[0])  # deg
    d_rudder = rad2deg(u[0])  # deg

    wing = aircraft['wing']
    ht = aircraft['horizontal']
    vt = aircraft['vertical']

    wing_aspect_ratio = wing['aspect_ratio']
    wing_area = wing['planform']
    wing_taper = wing['taper']
    wing_le_sweep = deg2rad(wing['sweep_LE'])
    wing_dihedral = deg2rad(wing['dihedral'])
    wing_root_le_pnt = avl.Point(wing['station'], wing['buttline'], wing['waterline'])
    wing_root_airfoil = wing['airfoils'][0]
    wing_tip_airfoil = wing['airfoils'][1]
    x_hinge_w = wing['control_1']['cf_c']

    ht_aspect_ratio = ht['aspect_ratio']
    ht_area = ht['planform']
    ht_taper = ht['taper']
    ht_sweep = deg2rad(ht['sweep_LE'])
    ht_dihedral = deg2rad(ht['dihedral'])
    ht_root_le_pnt = avl.Point(ht['station'], ht['buttline'], ht['waterline'])
    ht_root_airfoil = ht['airfoils'][0]
    ht_tip_airfoil = ht['airfoils'][0]
    x_hinge_ht = ht['control_1']['cf_c']
    cs_name_ht = ht['control_1']['name']

    wing_span = span(wing_aspect_ratio, wing_area)
    ht_span = span(ht_aspect_ratio, ht_area)

    wing_root_chord = root_chord(wing_aspect_ratio, wing_area, wing_taper)
    wing_tip_chord = wing_root_chord * wing_taper

    ht_root_chord = root_chord(ht_aspect_ratio, ht_area, ht_taper)
    ht_tip_chord = ht_root_chord * ht_taper

    wing_tip_le_pnt = avl.Point(x=wing_root_le_pnt[0] + x_mac(0.5 * wing_span, rad2deg(wing_le_sweep)),
                                y=wing_root_le_pnt[1] + 0.5 * wing_span,
                                z=wing_root_le_pnt[2] + 0.5 * wing_span * tan(wing_dihedral))

    root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                               chord=wing_root_chord,
                               airfoil=avl.NacaAirfoil(wing_root_airfoil))
    tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                              chord=wing_tip_chord,
                              airfoil=avl.NacaAirfoil(wing_tip_airfoil))

    # y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
    wing = avl.Surface(name='wing',
                       n_chordwise=12,
                       chord_spacing=avl.Spacing.cosine,
                       n_spanwise=20,
                       span_spacing=avl.Spacing.cosine,
                       y_duplicate=0.0,
                       sections=[root_section, tip_section])

    elevator = avl.Control(name=cs_name_ht,
                           gain=1,
                           x_hinge=x_hinge_ht,
                           duplicate_sign=1)

    ht_tip_le_pnt = avl.Point(x=ht_root_le_pnt.x + 0.5*ht_span*tan(ht_sweep),
                              y=ht_root_le_pnt.y + 0.5*ht_span,
                              z=ht_root_le_pnt.z + 0.5*ht_span*tan(ht_dihedral))

    root_section = avl.Section(leading_edge_point=ht_root_le_pnt,
                               chord=ht_root_chord,
                               airfoil=avl.NacaAirfoil(ht_root_airfoil),
                               controls=[elevator])
    tip_section = avl.Section(leading_edge_point=ht_tip_le_pnt,
                              chord=ht_tip_chord,
                              airfoil=avl.NacaAirfoil(ht_tip_airfoil),
                              controls=[elevator])
    horizontal_tail = avl.Surface(name='horizontal_tail',
                                  n_chordwise=12,
                                  chord_spacing=avl.Spacing.cosine,
                                  n_spanwise=20,
                                  span_spacing=avl.Spacing.cosine,
                                  y_duplicate=0.0,
                                  sections=[root_section, tip_section])

    wing_mac = mac(wing_aspect_ratio, wing_area, wing_taper)

    ref_pnt = avl.Point(aircraft['weight']['cg'][0], aircraft['weight']['cg'][1], aircraft['weight']['cg'][2])

    aircraft = avl.Geometry(name='aircraft',
                            reference_area=wing_area,
                            reference_chord=wing_mac,
                            reference_span=wing_span,
                            reference_point=ref_pnt,
                            mach=mach,
                            surfaces=[wing, horizontal_tail])

    # create a session with only the geometry
    session = avl.Session(geometry=aircraft)

    # check if we have ghostscript
    if 'gs_bin' in session.config.settings:
        img = session.save_geometry_plot()[0]
        avl.show_image(img)
    else:
        session.show_geometry()

    # create a function for showing the Trefftz plot, since we'll be using it more often
    def show_treffz(session_1):
        if 'gs_bin' in session_1.config.settings:
            images = session_1.save_trefftz_plots()
            for iimg in images:
                avl.show_image(iimg)
        else:
            for idx, _ in enumerate(session.cases):
                session_1.show_trefftz_plot(idx + 1)  # cases start from 1

    simple_case = avl.Case(name=case_name,
                           alpha=alpha,
                           beta=beta,
                           pitch_rate=pitch_rate * wing_mac / (2 * mach * 1116),
                           elevator=d_elevator)
    session = avl.Session(geometry=aircraft, cases=[simple_case])

    # show_treffz(session)

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

