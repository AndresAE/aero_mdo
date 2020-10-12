from matplotlib import pyplot as plt
from numpy import arctan, array, cos, deg2rad, linspace, min, ones, pi, sin, size, tan
from scipy.interpolate import interp1d
from scipy.optimize import minimize
from src.airplanes.example.plane import plane, requirements
from src.analysis.constraint import takeoff, master_constraint, stall_speed
from src.analysis.longitudinal import short_period_mode, static_margin
from src.analysis.lateral_directional import directional_stability, dutch_roll_mode, lateral_stability, \
    minimum_control_speed_air
from src.analysis.trim import trim_vr, trim_alpha_de_nonlinear
from src.common import Atmosphere
from src.modeling.aerodynamics import polhamus
from src.modeling import Fuselage, MassProperties, Propulsion


def horizontal_tail(plane, x, u, tol=10e-4):
    zeta_sp_req = 0.4
    dz = 1
    s_ht = plane['horizontal']['planform']
    a = Atmosphere(x[-1]).speed_of_sound()
    sm = static_margin(plane, x[0] / a)
    while abs(dz) > tol:
        plane['horizontal']['planform'] = s_ht
        zeta_sp, omega_sp, cap = short_period_mode(plane, x, u)
        dz = zeta_sp - zeta_sp_req
        s_ht = s_ht * (1 - float(dz))
    return s_ht


def landing_gear_location(plane, k=0.1):
    """return landing gear coordinates."""
    theta = deg2rad(plane['wing']['alpha_stall'] + 1)
    x_cg = plane['weight']['cg'][0]
    z_cg = plane['weight']['cg'][2]
    # fuselage impact points
    z_fus = min([plane['horizontal']['waterline'], plane['vertical']['waterline']])
    x_fus = max([plane['horizontal']['station'], plane['vertical']['station']])
    x_lg = (cos(theta) / sin(theta) * x_cg + z_cg - z_fus + sin(theta) / cos(theta) * x_fus) / (
                sin(theta) / cos(theta) + cos(theta) / sin(theta))
    h_lg = (x_lg - x_cg) / tan(theta) - z_cg
    x_ng = (x_cg - x_lg) / k + x_lg
    y_lg = plane['fuselage']['width'] / 2
    return x_ng, x_lg, y_lg, h_lg


def propulsion_sizing(plane, thrust, speed, altitude, tol=10e-4):
    prop = plane['propulsion']
    throttle = ones(plane['propulsion']['n_engines'])

    def obj(x):
        obj = abs(x[0]) + abs(x[1]) + abs(x[2]) / 100
        return obj

    def thrust_constraint(x):
        t = ones(size(speed))
        for jj in range(0, size(speed)):
            for ii in range(0, prop['n_engines']):
                plane['propulsion']["engine_%d" % (ii + 1)]['pitch'] = x[0]
                plane['propulsion']["engine_%d" % (ii + 1)]['diameter'] = x[1]
                plane['propulsion']["engine_%d" % (ii + 1)]['rpm_max'] = x[2]
            fm = Propulsion(plane['propulsion'], array([speed[jj], altitude[jj]]), throttle, array([0, 0, 0])).thrust_f_m()
            t[jj] = fm[0] - thrust[jj]
        return t / 1000

    lim = ([1, 20], [0.1, 10], [1000, 12000])
    x0 = array([5, 5, 10000])
    u_out = minimize(obj, x0, bounds=lim, tol=tol,
                     constraints=({'type': 'ineq', 'fun': thrust_constraint}),
                     options=({'maxiter': 200}))
    return u_out['x']


def vertical_tail(plane, x, u, tol=10e-4):
    zeta_dr_req = 0.4
    c_n_b_reg = 0.03
    alpha = arctan(x[2] / x[0])
    a = Atmosphere(x[-1]).speed_of_sound()
    dz = 1
    s_vt_dr = plane['vertical']['planform']
    while abs(dz) > tol:
        plane['vertical']['planform'] = s_vt_dr
        zeta_dr, omega_dr = dutch_roll_mode(plane, x, u)
        dz = zeta_dr - zeta_dr_req
        s_vt_dr = s_vt_dr * (1 - float(dz))
    s_vt_ds = plane['vertical']['planform']
    dc = 1
    while abs(dc) > tol:
        plane['vertical']['planform'] = s_vt_ds
        c_n_b = directional_stability(plane, x[0] / a, alpha)
        dc = c_n_b - c_n_b_reg
        s_vt_ds = s_vt_ds * (1 - float(dc))
    return max([s_vt_dr, s_vt_ds])


def wing_loading(plane, requirements):
    """return optimum wing and thrust loading based on requirements."""
    cla = polhamus(2 * pi, plane['wing']['aspect_ratio'], 0.3, plane['wing']['taper'], plane['wing']['sweep_LE'])
    cl_max = cla * deg2rad(plane['wing']['alpha_stall'])
    a = Atmosphere(requirements['performance']['to_altitude']).speed_of_sound()
    w_s = linspace(5, 100, 50)
    t_w_to = takeoff(plane, w_s, requirements['performance']['bfl'], requirements['performance']['to_altitude'], plane['landing_gear']['mu_roll'])
    w_s_s = stall_speed(requirements['performance']['stall_speed'] / a, requirements['performance']['to_altitude'], cl_max, 1, 0)
    t_w_c = master_constraint(plane, w_s, requirements['performance']['cruise_mach'], requirements['performance']['cruise_altitude'], 1, 0, 0)
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
    sm_reg = 0.05
    a = Atmosphere(altitude).speed_of_sound()
    dx = 1
    x_w = plane['wing']['station']
    while abs(dx) > tol:
        plane['wing']['station'] = x_w
        w, plane['weight']['cg'] = MassProperties(plane).weight_buildup(requirements)
        sm = static_margin(plane, v / a)
        dx = sm / 100 - sm_reg
        x_w = x_w - float(dx)
    return x_w, plane['weight']['cg']


# fixed
l_cab = Fuselage(plane).far_25_length()
plane['fuselage']['length'] = plane['fuselage']['length'] - plane['fuselage']['l_cabin'] + l_cab
a, b, dy, w = Fuselage(plane).cross_section()
plane['fuselage']['height'] = 2 * a
plane['fuselage']['width'] = 2 * b
plane['horizontal']['station'] = plane['fuselage']['length'] - 5
plane['vertical']['station'] = plane['fuselage']['length'] - 8
# iterative
plane['weight']['weight'], cg = MassProperties(plane).weight_buildup(requirements)
dw = 100
while abs(dw) > 10:
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
    out_prop = propulsion_sizing(plane, array([t, t, t]), array([100, 200, 300]), array([0, 20000, 20000]), tol=10e-4)
    for ii in range(0, plane['propulsion']['n_engines']):
        plane['propulsion']["engine_%d" % (ii + 1)]['pitch'] = out_prop[0]
        plane['propulsion']["engine_%d" % (ii + 1)]['diameter'] = out_prop[1]
        plane['propulsion']["engine_%d" % (ii + 1)]['rpm_max'] = out_prop[2]
    plane['wing']['station'],  plane['weight']['cg'] = wing_location(plane, requirements, 300, 20000)
    x_ng, x_mg, y_mg, l_g = landing_gear_location(plane)

    c = trim_alpha_de_nonlinear(plane, 300, 20000, 0)
    u = [0, deg2rad(c[0]), 0, 1]
    x = array([float(300 * cos(deg2rad(c[1]))), 0, float(300 * sin(deg2rad(c[1]))), 0, float(deg2rad(c[1])), 0, 0, 0, 0, 0, 0, 20000])
    plane['horizontal']['planform'] = horizontal_tail(plane, x, u)
    plane['vertical']['planform'] = vertical_tail(plane, x, u)
    plane['weight']['weight'], cg = MassProperties(plane).weight_buildup(requirements)
    dw = w_i - plane['weight']['weight']

a = 1
# elevator_sizing()
# rudder_sizing()
# aileron_sizing()


# def aileron_sizing():
#     roll_control
#     return ca_c, ba_b
#
#
# def elevator_sizing():
#     to_rotation
#     maneuvering
#     return ce_c
#
#
# def rudder_sizing():
#     vmc
#     crosswind
#     return ce_c
#
# def vertical_tail():
#     dutch_roll_damping
#     return s_v
# def wing_location():
#     return wing_station
