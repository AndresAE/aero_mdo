from numpy import array, arctan, cos, linalg, ones, sin, sqrt, rad2deg
from scipy.interpolate import interp2d
from src.common.rotations import body_to_wind
from src.modeling.aerodynamics import dynamic_pressure
from src.common import Atmosphere
from src.common.report_tools import load_aero_model
from src.modeling import Aircraft, Propulsion
from src.modeling.trapezoidal_wing import mac, span


def c_f_m(aircraft, x, u):
    """return aircraft body axis forces and moments."""
    s = aircraft['wing']['planform']  # [ft2]
    altitude = x[-1]  # [ft]
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    v = sqrt(x[0]**2 + x[1]**2 + x[2]**2)  # [ft/s]
    mach = v/a  # []
    alpha = arctan(x[2]/x[0])  # [rad]
    beta = arctan(x[1]/x[0])  # [rad]
    weight = array([0, 0, 0, 0, 0, 0])

    c_bar = mac(aircraft['wing']['aspect_ratio'], s, aircraft['wing']['taper'])  # [ft]
    b = span(aircraft['wing']['aspect_ratio'], s)  # [ft]
    throttle = u[3] * ones(aircraft['propulsion']['n_engines'])  # []

    # get thrust contributions
    c_f_m_t = Propulsion(aircraft['propulsion'], x, throttle, aircraft['weight']['cg']) .thrust_f_m()

    # get weight contributions
    weight[0:3] = aircraft['weight']['weight'] * array([-sin(x[4]), cos(x[4]) * sin(x[3]), cos(x[4]) * cos(x[3])])

    q_bar = dynamic_pressure(mach, altitude)  # [psf]
    s = aircraft['wing']['planform']  # [ft2]

    c_aero = linear_aero(aircraft, x, u)

    c = array([- c_aero[0], c_aero[1], - c_aero[2], c_aero[3]*b, c_aero[4]*c_bar, c_aero[5]*b])*q_bar*s
    b_2_w = body_to_wind(alpha, beta)
    c[0:3] = linalg.inv(b_2_w) @ c[0:3]
    c[3:6] = linalg.inv(b_2_w) @ c[3:6]
    c = c + c_f_m_t + weight
    return c


def landing_gear_loads(aircraft, x, c, fix=False, brake=0):
    """return landing gear loads."""
    v = (x[0]**2+x[1]**2+x[2]**2)**0.5
    rho = Atmosphere(x[-1]).air_density()
    q_bar = 0.5*rho*v**2
    # normal loads
    x_1 = aircraft['weight']['cg'][0] - aircraft['landing_gear']['nose'][0]
    x_2 = aircraft['weight']['cg'][0] - aircraft['landing_gear']['main'][0]
    z_1 = aircraft['weight']['cg'][2] - aircraft['landing_gear']['nose'][2]
    z_2 = aircraft['weight']['cg'][2] - aircraft['landing_gear']['main'][2]
    mu_b = aircraft['landing_gear']['mu_brake']
    mu_r = aircraft['landing_gear']['mu_roll']

    mu = (mu_b - mu_r) * brake + mu_r

    a = array([[x_1 - mu * z_1, (x_2 - mu * z_2)], [-1, -1]])
    b = array([[c[4]], [c[2]]])
    normal_loads = linalg.inv(a) @ b
    if fix:
        normal_loads[normal_loads > 0] = 0
    c_normal = array([0, 0, float(normal_loads[0] + normal_loads[1]),
                      0, float((normal_loads[0]*-x_1+normal_loads[1]*-x_2)), 0])
    # friction loads
    c_friction = array([float((normal_loads[0] + normal_loads[1]))*mu, 0, 0,
                        0, float((normal_loads[0]*z_1+normal_loads[1]*z_2))*mu, 0])
    # drag loads
    c_drag = array([aircraft['landing_gear']['c_d']*q_bar*aircraft['wing']['planform'], 0, 0, 0, 0, 0])
    c_gear = c_normal + c_drag + c_friction
    c_total = c + c_gear
    return c_total, c_gear, normal_loads


def linear_aero(aircraft, x, u):
    """return aircraft aero stability axis linear force and moment coefficients."""
    s = aircraft['wing']['planform']  # [ft2]
    altitude = x[-1]  # [ft]
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    v = sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)  # [ft/s]
    mach = v / a  # []
    alpha = arctan(x[2] / x[0])  # [rad]
    beta = arctan(x[1] / x[0])  # [rad]

    c_bar = mac(aircraft['wing']['aspect_ratio'], s, aircraft['wing']['taper'])  # [ft]
    b = span(aircraft['wing']['aspect_ratio'], s)  # [ft]

    p_hat = x[6] * b / (2 * v)  # []
    q_hat = x[7] * c_bar / (2 * v)  # []
    r_hat = x[8] * b / (2 * v)  # []

    d_aileron = u[0]  # [rad]
    d_elevator = u[1]  # [rad]
    d_rudder = u[2]  # [rad]

    alpha_dot = 0  # []

    ac = Aircraft(aircraft, mach)
    cd = (ac.c_d_zero(altitude) +
          ((ac.c_l_alpha() * alpha +
              ac.c_l_alpha_dot() * alpha_dot +
              ac.c_l_pitch_rate() * q_hat +
              ac.c_l_delta_elevator() * d_elevator) ** 2) /
          (ac.c_l_alpha() / 2 * aircraft['wing']['aspect_ratio']))

    cy = (ac.c_y_beta() * beta +
          ac.c_y_roll_rate(alpha) * p_hat +
          ac.c_y_yaw_rate(alpha) * r_hat +
          ac.c_y_delta_rudder() * d_rudder)

    cl = (ac.c_l_zero() +
          ac.c_l_alpha() * alpha +
          ac.c_l_alpha_dot() * alpha_dot +
          ac.c_l_pitch_rate() * q_hat +
          ac.c_l_delta_elevator() * d_elevator)

    cmr = (ac.c_r_beta(alpha) * beta +
           ac.c_r_roll_rate() * p_hat +
           ac.c_r_yaw_rate(alpha) * r_hat +
           ac.c_r_delta_aileron() * d_aileron +
           ac.c_r_delta_rudder() * d_rudder)

    cmp = (ac.c_m_zero(altitude) +
           ac.c_m_alpha() * alpha +
           ac.c_m_alpha_dot() * alpha_dot +
           ac.c_m_pitch_rate() * q_hat +
           ac.c_m_delta_elevator() * d_elevator)

    cmy = (ac.c_n_beta(alpha) * beta +
           ac.c_n_roll_rate(alpha) * p_hat +
           ac.c_n_yaw_rate(alpha) * r_hat +
           ac.c_n_delta_aileron() * d_aileron +
           ac.c_n_delta_rudder() * d_rudder)
    c_aero = array([cd, cy, cl, cmr, cmp, cmy])
    return c_aero


def nonlinear_aero(aircraft, x, u):
    """return aircraft aero stability axis nonlinear force and moment coefficients."""
    altitude = x[-1]  # [ft]
    a = Atmosphere(altitude).speed_of_sound()  # [ft/s]
    v = sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)  # [ft/s]
    mach = v / a  # []
    alpha = rad2deg(arctan(x[2] / x[0]))
    beta = rad2deg(arctan(x[1] / x[0]))
    model = load_aero_model(aircraft['name'])

    p = rad2deg(x[6])
    q = rad2deg(x[7])
    r = rad2deg(x[8])

    d_aileron = rad2deg(u[0])  # [rad]
    d_elevator = rad2deg(u[1])  # [rad]
    d_rudder = rad2deg(u[2])  # [rad]

    names = ['cd', 'cy', 'cl', 'cmr', 'cmp', 'cmy']
    c = []
    for cfm in names:
        f_bas = interp2d(model['baseline']['alpha'], model['baseline']['mach'], model['baseline']['cfm'][cfm])
        f_lat = interp2d(model['lat_dir']['beta'], model['lat_dir']['mach'], model['lat_dir']['cfm'][cfm])
        f_ail = interp2d(model['aileron']['d_aileron'], model['aileron']['mach'], model['aileron']['cfm'][cfm])
        f_ele = interp2d(model['elevator']['d_elevator'], model['elevator']['mach'], model['elevator']['cfm'][cfm])
        f_rud = interp2d(model['rudder']['d_rudder'], model['rudder']['mach'], model['rudder']['cfm'][cfm])
        f_p = interp2d(model['p']['p'], model['p']['mach'], model['p']['cfm'][cfm])
        f_q = interp2d(model['q']['q'], model['q']['mach'], model['q']['cfm'][cfm])
        f_r = interp2d(model['r']['r'], model['r']['mach'], model['r']['cfm'][cfm])
        c.append(float(f_bas(alpha, mach)) +
                 float(f_lat(beta, mach) - f_bas(0, mach)) +
                 float(f_ail(d_aileron, mach) - f_bas(0, mach)) +
                 float(f_ele(d_elevator, mach) - f_bas(0, mach)) +
                 float(f_rud(d_rudder, mach) - f_bas(0, mach)) +
                 float(f_p(p, mach) - f_bas(0, mach)) +
                 float(f_q(q, mach) - f_bas(0, mach)) +
                 float(f_r(r, mach) - f_bas(0, mach)))
    ac = Aircraft(aircraft, mach)
    c_aero = (array(c) + array([ac.c_d_zero(altitude), 0, 0, 0, 0, 0]))
    return c_aero
