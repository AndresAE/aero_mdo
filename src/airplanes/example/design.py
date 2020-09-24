from matplotlib import pyplot as plt
from numpy import arctan, array, cos, deg2rad, rad2deg, sin, zeros
from src.common import Atmosphere
from src.common import Earth
from src.analysis.lateral_directional import directional_stability, lateral_stability, dutch_roll_mode, plot_dr, \
    roll_mode, spiral_mode
from src.analysis.longitudinal import aircraft_range, balanced_field_length, maneuvering, short_period_mode, plot_sp, \
    static_margin, specific_excess_power
from src.analysis.trim import trim_aileron, trim_aileron_rudder, trim_alpha_de
import os
g = Earth(0).gravity()  # f/s2

# Requirements
n_z = [-1, 1, 2.5]  # [g]
crosswind = 15  # [ft/s]
p = 15  # [deg/s]
h_to = 5000  # [ft]


# Sweep
def report_sweep(plane, requirements, name=''):
    cwd = os.getcwd()
    path = cwd + '/src/airplanes/'+name+'/output'
    os.mkdir(path)
    machs = array([0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6])
    altitudes = array([0, 5000, 10000, 15000, 20000, 25000, 30000, 35000])

    # set arrays
    cap = zeros((len(altitudes), len(machs)))
    zeta_sp = zeros((len(altitudes), len(machs)))
    zeta_dr = zeros((len(altitudes), len(machs)))
    omega_dr = zeros((len(altitudes), len(machs)))
    p_s = zeros((len(altitudes), len(machs)))
    alpha_nz = zeros((len(altitudes), len(machs), len(n_z)))
    de_nz = zeros((len(altitudes), len(machs), len(n_z)))
    sm = zeros((len(altitudes), len(machs)))
    c_n = zeros((len(altitudes), len(machs)))
    c_r = zeros((len(altitudes), len(machs)))
    t_roll = zeros((len(altitudes), len(machs)))
    t_2_d_sp = zeros((len(altitudes), len(machs)))
    dr_beta = zeros((len(altitudes), len(machs)))
    da_roll = zeros((len(altitudes), len(machs)))
    r = zeros((len(altitudes), len(machs)))

    # iterative methods
    i_alt = 0
    for alt_i in altitudes:
        i_mach = 0
        for mach_i in machs:
            a = Atmosphere(alt_i).speed_of_sound()
            v = mach_i * a
            trim_out = trim_alpha_de(plane, v, alt_i, 0)
            aoa = deg2rad(trim_out[0])  # rad
            u = v * cos(aoa)  # ft/s
            w = v * sin(aoa)  # ft/s
            de = deg2rad(trim_out[1])  # rad
            x_0 = array([float(u), 0.0, float(w), 0.0, float(aoa), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt_i])
            u_0 = array([0.0, float(de), 0.0, 1])
            beta = rad2deg(arctan(crosswind / v))

            # stability and control
            omega, zeta_sp[i_alt, i_mach], cap[i_alt, i_mach] = short_period_mode(plane, x_0, u_0)
            omega_dr[i_alt, i_mach], zeta_dr[i_alt, i_mach] = dutch_roll_mode(plane, x_0, u_0)
            t_2_d_sp[i_alt, i_mach] = spiral_mode(plane, x_0, u_0)
            t_roll[i_alt, i_mach] = roll_mode(plane, x_0, u_0)
            sm[i_alt, i_mach] = static_margin(plane, mach_i)
            c_n[i_alt, i_mach] = directional_stability(plane, mach_i, aoa)
            c_r[i_alt, i_mach] = lateral_stability(plane, mach_i, aoa)
            out_beta = trim_aileron_rudder(plane, v, alt_i, float(aoa), beta, 0, 0)
            dr_beta[i_alt, i_mach] = out_beta[1]
            out_p = trim_aileron(plane, v, alt_i, p)
            da_roll[i_alt, i_mach] = out_p

            # performance
            p_s[i_alt, i_mach] = specific_excess_power(plane, x_0, u_0)
            alpha_nz[i_alt, i_mach, :], de_nz[i_alt, i_mach, :] = maneuvering(plane, mach_i, alt_i, n_z)
            r[i_alt, i_mach] = aircraft_range(plane, mach_i, alt_i)

            i_mach = i_mach + 1
        i_alt = i_alt + 1

    # non-iterative methods
    x_0 = array([float(0.01), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, h_to])
    u_0 = array([0.0, 0.0, 0.0, 1])
    balanced_field_length(plane, x_0, u_0)
    plt.savefig(path + '/balanced_field_length.png')
    plt.close()

    # plotting
    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, sm[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Static Margin')
    plt.grid(True)
    plt.legend()
    plt.ylabel('SM [%MAC]')
    plt.xlabel('Mach')
    plt.savefig(path + '/static_margin.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, c_n[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Directional Stability')
    plt.grid(True)
    plt.legend()
    plt.ylabel('CNBeta [1/rad]')
    plt.xlabel('Mach')
    plt.savefig(path + '/directional_stability.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, c_r[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('lateral Stability')
    plt.grid(True)
    plt.legend()
    plt.ylabel('CLBeta [1/rad]')
    plt.xlabel('Mach')
    plt.savefig(path + '/lateral_stability.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(omega_dr[i_alt, :], zeta_dr[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
        plt.ylim((0, 1))
    plt.title("Dutch-Roll Damping")
    plt.grid(True)
    plt.legend()
    plot_dr()
    plt.savefig(path + '/dutch_roll.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(zeta_sp[i_alt, :], cap[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
        plt.ylim((0, 1))
    plt.title("Short-Period Damping")
    plt.grid(True)
    plt.legend()
    plot_sp()
    plt.savefig(path + '/short_period.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, t_roll[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Roll Mode')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Roll Time Constant [sec]')
    plt.xlabel('Mach')
    plt.savefig(path + '/roll_mode.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, t_2_d_sp[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Spiral Mode')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Time To Double [sec]')
    plt.xlabel('Mach')
    plt.savefig(path + '/spiral_mode.png')
    plt.close()

    for i_alt in range(0, len(altitudes)):
        fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(12, 6))
        for ni in range(0, len(n_z)):
            axes[0].plot(machs, de_nz[i_alt, :, ni], label="Nz %d g" % (n_z[ni]))
            axes[1].plot(machs, alpha_nz[i_alt, :, ni], label="Nz %d g" % (n_z[ni]))
        axes[0].set_title("Maneuver Capability, %d ft" % (altitudes[i_alt]))
        axes[0].set_xlabel('Mach')
        axes[0].set_ylabel('Elevator Deflection [deg]')
        axes[0].legend()
        axes[0].grid(True)
        axes[1].set_xlabel('Mach')
        axes[1].set_ylabel('Angle of Attack [deg]')
        axes[1].legend()
        axes[1].grid(True)
        plt.savefig(path + "/maneuver_capability_%d.png" % (altitudes[i_alt]))
        plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, dr_beta[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Sideslip')
    plt.grid(True)
    plt.legend()
    plt.ylabel('rudder [deg]')
    plt.xlabel('Mach')
    plt.savefig(path + '/sideslip_capability.png')
    plt.close()

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, da_roll[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Roll Control')
    plt.grid(True)
    plt.legend()
    plt.ylabel('aileron [deg]')
    plt.xlabel('Mach')
    plt.savefig(path + '/roll_control.png')
    plt.close()

    plt.figure()
    levels = [0, 100, 500, 1000]
    cs = plt.contour(machs, altitudes, p_s, levels)
    plt.clabel(cs, levels, fmt='%1.0f')
    plt.title('Specific Excess Power (fpm)')
    plt.grid(True)
    plt.xlabel('Mach')
    plt.ylabel('Altitude [ft]')
    plt.savefig(path + '/specific_excess_power.png')
    plt.close()

    plt.figure()
    levels = [2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000]
    cs = plt.contour(machs, altitudes, r, levels)
    plt.clabel(cs, levels, fmt='%1.0f')
    plt.title('range [nm]')
    plt.grid(True)
    plt.xlabel('Mach')
    plt.ylabel('Altitude [ft]')
    plt.savefig(path + '/range.png')
    plt.close()
