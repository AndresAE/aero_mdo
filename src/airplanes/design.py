from matplotlib import pyplot as plt
from numpy import arctan, array, cos, deg2rad, linspace, min, ones, pi, rad2deg, sin, size, tan
from scipy.interpolate import interp1d
from scipy.optimize import minimize, Bounds
from src.airplanes.visualization import print_plane
from src.analysis.constraint import takeoff, master_constraint, stall_speed
from src.analysis.longitudinal import short_period_mode, static_margin
from src.analysis.lateral_directional import directional_stability, dutch_roll_mode
from src.analysis.trim import trim_alpha_de_nonlinear
from src.common import Atmosphere, Earth, constants
from src.modeling.aerodynamics import polhamus
from src.modeling import Aircraft, Fuselage, MassProperties, Propulsion, trapezoidal_wing
from src.modeling.force_model import c_f_m, landing_gear_loads
g = Earth(0).gravity()


def dihedral(plane, req):
    y_lg = plane['landing_gear']['main'][1]
    phi = deg2rad(req['stability_and_control']['lto_roll_angle'])
    for ii in range(0, plane['propulsion']['n_engines']):
        b = trapezoidal_wing.span(plane['wing']['aspect_ratio'], plane['wing']['planform'])
        z_tip = (b / 2 - y_lg) * tan(phi) - plane['wing']['waterline']
        gamma = max([rad2deg(arctan(z_tip / (b / 2))), 0])
    return gamma


def elevator(plane, req, tol=10e-1):
    alt = req['flight_envelope']['altitude'][1]
    alpha = plane['wing']['alpha_stall']
    de = deg2rad(plane['horizontal']['control_1']['limits'][0])
    w = plane['weight']['weight']
    plane['weight']['weight'] = w * req['loads']['n_z'][1]

    def obj(x):
        return x[0]

    def long_constraint(x):
        v = x[0]
        plane['horizontal']['control_1']['cf_c'] = x[1]
        s = array([float(v * cos(deg2rad(alpha))), 0, float(v * sin(deg2rad(alpha))),
                   0, float(deg2rad(alpha)), 0, 0, 0, 0, 0, 0, alt])
        u = [0, de, 0, 0.01]
        cfm = c_f_m(plane, s, u)
        return abs(cfm[2]) + abs(cfm[4])

    lim = ([5, 1000], [0, 1])
    x0 = array([10, plane['horizontal']['control_1']['cf_c']])
    u_out_1 = minimize(obj, x0, bounds=lim, tol=tol,
                       constraints=({'type': 'eq', 'fun': long_constraint}),
                       options=({'maxiter': 200}))
    plane['weight']['weight'] = w
    u = [0, de, 0, 0.01]
    vr = req['performance']['stall_speed'] * 1.15

    def obj(x):
        return x[0]

    def v_stab(x):
        plane['horizontal']['control_1']['cf_c'] = x[0]
        s = array([vr, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        c = c_f_m(plane, s, u)
        c_t, c_g, normal_loads = landing_gear_loads(plane, s, c)
        return float(normal_loads[0])

    lim = Bounds(0, 1)
    x0 = array([plane['horizontal']['control_1']['cf_c']])
    u_out_2 = minimize(obj, x0, bounds=lim, tol=tol,
                       constraints=({'type': 'eq', 'fun': v_stab}),
                       options=({'maxiter': 200}))

    return max([u_out_1['x'][1], u_out_2['x'][0]])


def engine_height(plane, req):
    y_lg = plane['landing_gear']['main'][1]
    phi = deg2rad(req['stability_and_control']['lto_roll_angle'])
    for ii in range(0, plane['propulsion']['n_engines']):
        y_eng = plane['propulsion']["engine_%d" % (ii + 1)]['buttline']
        d = plane['propulsion']["engine_%d" % (ii + 1)]['diameter']
        z_eng = (abs(y_eng) - y_lg) * tan(phi) + d / 2
        plane['propulsion']["engine_%d" % (ii + 1)]['waterline'] = max([
            plane['propulsion']["engine_%d" % (ii + 1)]['waterline'], z_eng])
    return


def landing_gear_location(plane, k=0.07, mount='fuselage'):
    """return landing gear coordinates."""
    theta = deg2rad(plane['wing']['alpha_stall'] + 1)
    x_cg = plane['weight']['cg'][0]
    z_cg = plane['weight']['cg'][2]
    # fuselage impact points
    z_fus = min([plane['horizontal']['waterline'], plane['vertical']['waterline']])
    x_fus = max([plane['horizontal']['station'], plane['vertical']['station'], plane['fuselage']['length']])
    x_lg = (cos(theta) / sin(theta) * x_cg + z_cg - z_fus + sin(theta) / cos(theta) * x_fus) / (
            sin(theta) / cos(theta) + cos(theta) / sin(theta))
    h_lg = (x_lg - x_cg) / tan(theta) - z_cg
    x_ng = (x_cg - x_lg) / k + x_lg
    if mount == 'fuselage':
        y_lg = plane['fuselage']['width'] / 2 + 1
    elif mount == 'nacelle':
        y_lg = plane['propulsion']['engine_1']['buttline']

    return x_ng, x_lg, y_lg, h_lg


def longitudinal_sizing(plane, req, s, u, tol=10e-1):
    v = (s[0] ** 2 + s[2] ** 2) ** 0.5
    a = Atmosphere(s[-1]).speed_of_sound()
    zeta_req = req['stability_and_control']['zeta_sp']
    sm_req = req['stability_and_control']['sm']*100

    de = deg2rad(plane['horizontal']['control_1']['limits'][0])
    u_r = [0, de, 0, 0.01]
    vr = req['performance']['stall_speed'] * 1.15
    alt_to = req['performance']['to_altitude']

    def obj(x):
        return x[1]

    def sm_constraint(x):
        plane['wing']['station'] = x[0]
        plane['horizontal']['planform'] = x[1]
        plane['weight']['weight'], plane['weight']['cg'] = MassProperties(plane).weight_buildup(req)
        sm = static_margin(plane, v / a)
        c = sm - sm_req
        return c

    def sp_constraint(x):
        plane['wing']['station'] = x[0]
        plane['horizontal']['planform'] = x[1]
        plane['weight']['weight'], plane['weight']['cg'] = MassProperties(plane).weight_buildup(req)
        zeta_sp, omega_sp, cap = short_period_mode(plane, s, u)
        plane['horizontal']['control_1']['cf_c'] = 0.99
        s_r = array([vr, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, alt_to])
        cfm = c_f_m(plane, s_r, u_r)
        c_t, c_g, normal_loads = landing_gear_loads(plane, s_r, cfm)
        c = array([zeta_sp - zeta_req, float(normal_loads[0])])
        return c

    lim = ([0, plane['fuselage']['length']], [0.1, plane['wing']['planform']])
    x0 = array([plane['wing']['station'], plane['horizontal']['planform']])
    eq_con = {'type': 'eq', 'fun': sm_constraint}
    ineq_con = {'type': 'ineq', 'fun': sp_constraint}
    u_out = minimize(obj, x0, bounds=lim, tol=tol, constraints=[eq_con, ineq_con], options=({'maxiter': 200}))
    return u_out['x']


def propulsion_sizing(plane, thrust, speed, altitude, tol=10e-4):
    prop = plane['propulsion']
    throttle = ones(plane['propulsion']['n_engines'])

    def obj(x):
        return x[1]*x[2]/100

    def thrust_constraint(x):
        t = ones(size(speed))
        for jj in range(0, size(speed)):
            for ii in range(0, prop['n_engines']):
                plane['propulsion']["engine_%d" % (ii + 1)]['pitch'] = x[0]
                plane['propulsion']["engine_%d" % (ii + 1)]['diameter'] = x[1]
                plane['propulsion']["engine_%d" % (ii + 1)]['rpm_max'] = x[2]*1000
            fm = Propulsion(plane['propulsion'], array([speed[jj], altitude[jj]]),
                            throttle, array([0, 0, 0])).thrust_f_m()
            t[jj] = fm[0] - thrust[jj]
        return min(t) / 1000

    lim = ([5, 35], [0.01, 10], [1, 8])
    x0 = array([20, 1, 2000])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'ineq', 'fun': thrust_constraint}),
                     options=({'maxiter': 200}))
    return u_out['x']


def range_iter(plane, req):
    sigma = plane['propulsion']['energy_density']
    eta = plane['propulsion']['total_efficiency']
    r = req['performance']['range']
    m_cruise = req['performance']['cruise_mach']
    alt_cruise = req['performance']['cruise_altitude']
    hold = 5 * 60  # [sec]
    divert = 45 * 60  # [sec]
    e = plane['wing']['aspect_ratio'] / (2 + plane['wing']['aspect_ratio'])
    l_d = 0.5 * (pi * plane['wing']['aspect_ratio'] * e / Aircraft(plane, m_cruise).c_d_zero(alt_cruise))**0.5
    w = plane['weight']['weight']
    delta = 10
    fuel_weight = 0
    while delta > 1:
        gamma = 10  # [deg]
        m_fuel_climb = w / (l_d * cos(deg2rad(gamma)) * sigma * eta) * (alt_cruise / tan(deg2rad(gamma)))
        m_fuel_cruise = w * (r * constants.ft2nm()) / (l_d * sigma * eta)
        m_fuel_divert = w * (hold + divert) * (Atmosphere(alt_cruise).speed_of_sound() * m_cruise) / (l_d * sigma * eta)
        fuel_weight = g * (m_fuel_climb * 3 + m_fuel_cruise + m_fuel_divert) * 1.1
        w_out = plane['weight']['weight'] - plane['propulsion']['fuel_mass'] * g + fuel_weight
        delta = abs(w - w_out)
        w = w_out
    return fuel_weight


def rudder(plane, req, tol=10e-1):
    alt = req['flight_envelope']['altitude'][1]
    alpha = plane['wing']['alpha_stall'] + 2
    dr = deg2rad(plane['vertical']['control_1']['limits'][0])

    def obj(x):
        return x[0]

    def oei_constraint(x):
        v = x[0]
        plane['vertical']['control_1']['cf_c'] = x[1]
        s = array([float(v * cos(deg2rad(alpha))), 0, float(v * sin(deg2rad(alpha))),
                   0, float(deg2rad(alpha)), 0, 0, 0, 0, 0, 0, 0])
        u = [0, deg2rad(x[2]), dr, 1]
        cfm = c_f_m(plane, s, u, engine_out=True)
        return abs(cfm[2])+abs(cfm[4])+abs(cfm[5])

    lim = ([5, 1000], [0, 1], plane['horizontal']['control_1']['limits'])
    x0 = array([10, plane['horizontal']['control_1']['cf_c'], 0])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'eq', 'fun': oei_constraint}),
                     options=({'maxiter': 200}))

    return u_out['x'][1]


def vertical_tail(plane, req, s, u, tol=10e-4):
    zeta_dr_req = req['stability_and_control']['zeta_dr']
    c_n_b_req = req['stability_and_control']['c_n_b']
    alpha = arctan(s[2] / s[0])
    a = Atmosphere(s[-1]).speed_of_sound()

    def obj(x):
        return x[0]

    def constraint(x):
        plane['vertical']['planform'] = x[0]
        c_n_b = directional_stability(plane, s[0] / a, alpha)
        zeta_dr, omega_dr = dutch_roll_mode(plane, s, u)
        c = array([zeta_dr - zeta_dr_req, c_n_b - c_n_b_req])
        return c

    lim = Bounds(0.1, float(plane['wing']['planform']))
    x0 = array([float(plane['vertical']['planform'])])
    ineq_con = {'type': 'ineq', 'fun': constraint}
    u_out = minimize(obj, x0, bounds=lim, tol=tol, constraints=ineq_con, options=({'maxiter': 200}))
    return u_out['x'][0]


def wing_loading(plane, requirements):
    """return optimum wing and thrust loading based on requirements."""
    cla = polhamus(2 * pi, plane['wing']['aspect_ratio'], 0.3, plane['wing']['taper'], plane['wing']['sweep_LE'])
    cl_max = cla * deg2rad(plane['wing']['alpha_stall'])
    a = Atmosphere(requirements['performance']['to_altitude']).speed_of_sound()
    w_s = linspace(5, 100, 50)
    t_w_to = takeoff(plane, w_s, requirements['performance']['bfl'], requirements['performance']['to_altitude'],
                     plane['landing_gear']['mu_roll'])
    w_s_s = stall_speed(requirements['performance']['stall_speed'] / a, requirements['performance']['to_altitude'],
                        cl_max, 1, 0)
    t_w_c = master_constraint(plane, w_s, requirements['performance']['cruise_mach'],
                              requirements['performance']['cruise_altitude'], 1, 0, 0)
    t_w_all = array([[t_w_c], [t_w_to]])
    t_w_max = t_w_all.max(axis=0)
    t_w_opt = min(t_w_max)
    f = interp1d(t_w_max[0, :], w_s)
    w_s_opt = f(t_w_opt)

    plt.figure()
    plt.plot(w_s_opt, t_w_opt, 'or', label='optimal point')
    plt.plot(w_s, t_w_to, label='takeoff')
    plt.plot(w_s, t_w_c, label='cruise')
    plt.plot([w_s_s, w_s_s], [0, 1])
    return w_s_opt, t_w_opt


def wing_location(plane, requirements, v, altitude, tol=10e-4):
    sm_reg = requirements['stability_and_control']['sm']
    a = Atmosphere(altitude).speed_of_sound()
    dx = 1
    x_w = plane['wing']['station']
    while abs(dx) > tol:
        plane['wing']['station'] = x_w
        for ii in range(0, plane['propulsion']['n_engines']):
            plane['propulsion']["engine_%d" % (ii + 1)]['station'] = x_w
        w, plane['weight']['cg'] = MassProperties(plane).weight_buildup(requirements)
        sm = static_margin(plane, v / a)
        dx = sm / 100 - sm_reg
        x_w = x_w - float(dx)
    return x_w, plane['weight']['cg']


def design(plane, requirements,
           wing_height='high', tail='conventional', engine='wing_mounted', landing_gear='fuselage', propulsion='h2'):

    if propulsion == 'h2':
        plane['propulsion']['energy_density'] = constants.energy_density_h2() * 2655224 / 0.0685218
        plane['propulsion']['total_efficiency'] = constants.eta_electric()
    elif propulsion == 'battery':
        plane['propulsion']['energy_density'] = constants.energy_density_li_s() * 2655224 / 0.0685218
        plane['propulsion']['total_efficiency'] = constants.eta_electric()
    elif propulsion == 'turboprop':
        plane['propulsion']['energy_density'] = constants.energy_density_jet_a() * 2655224 / 0.0685218
        plane['propulsion']['total_efficiency'] = constants.eta_turboprop()

    l_cab = Fuselage(plane).far_25_length()
    plane['fuselage']['length'] = plane['fuselage']['length'] - plane['fuselage']['l_cabin'] + l_cab
    plane['fuselage']['l_cabin'] = l_cab
    a, b, dy, w = Fuselage(plane).cross_section()
    plane['fuselage']['height'] = 2 * a
    plane['fuselage']['width'] = 2 * b
    plane['weight']['weight'], cg = MassProperties(plane).weight_buildup(requirements)
    plane['horizontal']['station'] = plane['fuselage']['length'] - 3
    plane['vertical']['station'] = plane['fuselage']['length'] - 5
    plane['horizontal']['waterline'] = plane['fuselage']['height']-2
    plane['vertical']['waterline'] = plane['fuselage']['height']-2
    if wing_height == 'low':
        plane['wing']['waterline'] = 0
    else:
        plane['wing']['waterline'] = plane['fuselage']['height']

    if tail == 'T':
        plane['vertical']['taper'] = 0.7

    v_cruise = (requirements['performance']['cruise_mach'] *
                Atmosphere(requirements['performance']['cruise_altitude']).speed_of_sound())
    v_stall = requirements['performance']['stall_speed']

    dw = 100
    # iterative
    while abs(dw) > 10:
        print('dw = %d' % dw)
        w_i = plane['weight']['weight']
        plane['weight']['weight'], cg = MassProperties(plane).weight_buildup(requirements)
        i_xx = MassProperties(plane).i_xx_simple()
        i_yy = MassProperties(plane).i_yy_simple()
        i_zz = MassProperties(plane).i_zz_simple()
        i_xz = MassProperties(plane).i_xz_simple()
        w_s, t_w = wing_loading(plane, requirements)
        plane['weight']['inertia'] = [[i_xx, 0, i_xz], [0, i_yy, 0], [i_xz, 0, i_zz]]
        plane['wing']['planform'] = plane['weight']['weight'] / w_s
        t = plane['weight']['weight'] * t_w

        thrust = array([t * Atmosphere(0).air_density() / 0.00238,
                        t * Atmosphere(requirements['performance']['cruise_altitude']).air_density() / 0.00238,
                        t * Atmosphere(requirements['performance']['cruise_altitude']).air_density() / 0.00238])
        out_prop = propulsion_sizing(plane, thrust, array([v_stall, v_cruise - 200, v_cruise]),
                                     array([0, requirements['performance']['cruise_altitude'],
                                            requirements['performance']['cruise_altitude']]), tol=10e-4)
        for ii in range(0, plane['propulsion']['n_engines']):
            plane['propulsion']["engine_%d" % (ii + 1)]['pitch'] = out_prop[0]
            plane['propulsion']["engine_%d" % (ii + 1)]['diameter'] = out_prop[1]
            plane['propulsion']["engine_%d" % (ii + 1)]['rpm_max'] = out_prop[2]*1000

        plane['wing']['station'],  plane['weight']['cg'] = wing_location(plane, requirements, 300, 20000)

        if tail == 'T':
            plane['horizontal']['waterline'] = plane['vertical']['waterline'] + trapezoidal_wing.span(
                plane['vertical']['aspect_ratio'], plane['vertical']['planform'], mirror=0)
        elif tail == 'conventional':
            plane['horizontal']['waterline'] = plane['vertical']['waterline']

        engine_height(plane, requirements)
        plane['wing']['dihedral'] = dihedral(plane, requirements)
        if engine == 'wing_mounted':
            for ii in range(0, plane['propulsion']['n_engines']):
                plane['propulsion']["engine_%d" % (ii + 1)]['station'] = plane['wing']['station']
        elif engine == 'fuselage_mounted':
            for ii in range(0, plane['propulsion']['n_engines']):
                plane['propulsion']["engine_%d" % (ii + 1)]['station'] = l_cab + plane['fuselage']['l_cockpit'] + 5

        x_ng, x_mg, y_mg, l_g = landing_gear_location(plane, mount=landing_gear)
        plane['landing_gear']['nose'] = [x_ng, 0, -l_g]
        plane['landing_gear']['main'] = [x_mg, y_mg, -l_g]

        plane['horizontal']['station'] = min([x_mg +
                                              (plane['horizontal']['waterline'] + l_g) /
                                              tan(deg2rad(plane['wing']['alpha_stall']+2)),
                                              plane['fuselage']['length']-3])
        plane['vertical']['station'] = plane['horizontal']['station'] - 2

        c = trim_alpha_de_nonlinear(plane, v_cruise, requirements['performance']['cruise_altitude'], 0)
        u = array([0, deg2rad(c[0]), 0, 1])
        x = array([float(v_cruise * cos(deg2rad(c[1]))), 0, float(v_cruise * sin(deg2rad(c[1]))),
                   0, float(deg2rad(c[1])), 0, 0, 0, 0, 0, 0, requirements['performance']['cruise_altitude']])
        out = longitudinal_sizing(plane, requirements, x, u)
        plane['wing']['station'] = out[0]
        plane['horizontal']['planform'] = out[1]
        plane['vertical']['planform'] = vertical_tail(plane, requirements, x, u)
        plane['propulsion']['fuel_mass'] = range_iter(plane, requirements) / g
        plane['weight']['weight'], cg = MassProperties(plane).weight_buildup(requirements)
        dw = w_i - plane['weight']['weight']

    plane['horizontal']['control_1']['cf_c'] = elevator(plane, requirements)
    rudder(plane, requirements)
    print_plane(plane)
    return plane
