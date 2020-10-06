from matplotlib import pyplot as plt
from numpy import deg2rad, linspace, pi
from src.airplanes.example.plane import plane, requirements
from src.analysis.constraint import takeoff, master_constraint, stall_speed
from src.common import Atmosphere
from src.modeling.aerodynamics import polhamus


def landing_gear_location():
    return x_ng, x_mg, y_mg, l_g


def horizontal_tail():
    return s_ht


def vertical_tail():
    return s_v


def wing_loading(plane, requirements):
    cla = polhamus(2 * pi, plane['wing']['aspect_ratio'], 0.3, plane['wing']['taper'], plane['wing']['sweep_LE'])
    cl_max = cla * deg2rad(plane['wing']['alpha_stall'])
    a = Atmosphere(requirements['performance']['to_altitude']).speed_of_sound()
    w_s = linspace(5, 100, 50)
    t_w_to = takeoff(plane, w_s, requirements['performance']['bfl'], requirements['performance']['to_altitude'], plane['landing_gear']['mu_roll'])
    w_s_s = stall_speed(requirements['performance']['stall_speed'] / a, requirements['performance']['to_altitude'], cl_max, 1, 0)
    t_w_c = master_constraint(plane, w_s, requirements['performance']['cruise_mach'], requirements['performance']['cruise_altitude'], 1, 0, 0)

    plt.figure()
    plt.plot(w_s, t_w_to)
    plt.plot(w_s, t_w_c)
    plt.plot([w_s_s, w_s_s], [0, 1])
    plt.show()
    return w_s_optimal, t_w_optimal


def wing_location():
    return wing_station
