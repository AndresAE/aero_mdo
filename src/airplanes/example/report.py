from matplotlib import pyplot as plt
from numpy import arctan, array, cos, deg2rad, linspace, sin, zeros
from src.common import Atmosphere, Earth
from src.common.report_tools import create_output_dir, plot_or_save
from src.analysis.lateral_directional import dutch_roll_mode, latdir_stability_nonlinear, plot_dr, roll_mode, \
    spiral_mode
from src.analysis.longitudinal import aircraft_range, balanced_field_length, maneuvering, plot_sp, short_period_mode, \
    specific_excess_power, static_margin_nonlinear
from src.analysis.trim import trim_aileron_nonlinear, trim_aileron_rudder_nonlinear, trim_alpha_de_nonlinear
g = Earth(0).gravity()  # f/s2
show_plot = 0
save_plot = 1


# Sweep
def report_sweep(plane, requirements):
    name = plane['name']
    create_output_dir(name)
    machs = linspace(requirements['flight_envelope']['mach'][0],
                     requirements['flight_envelope']['mach'][1], 5)
    altitudes = linspace(requirements['flight_envelope']['altitude'][0],
                         requirements['flight_envelope']['altitude'][1], 3)

    # Requirements
    n_z = [requirements['loads']['n_z'][0], 1, requirements['loads']['n_z'][1]]  # [g]
    crosswind = requirements['stability_and_control']['crosswind']  # [ft/s]
    p = deg2rad(requirements['stability_and_control']['roll_rate'])  # [rad/s]
    h_to = requirements['performance']['to_altitude']  # [ft]

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
    da_beta = zeros((len(altitudes), len(machs)))
    da_roll = zeros((len(altitudes), len(machs)))
    r = zeros((len(altitudes), len(machs)))

    # iterative methods
    i_alt = 0
    for alt_i in altitudes:
        i_mach = 0
        for mach_i in machs:
            a = Atmosphere(alt_i).speed_of_sound()
            v = mach_i * a
            trim_out = trim_alpha_de_nonlinear(plane, v, alt_i, 0)
            aoa = deg2rad(trim_out[0])  # rad
            u = v * cos(aoa)  # ft/s
            w = v * sin(aoa)  # ft/s
            de = deg2rad(trim_out[1])  # rad
            x_0 = array([float(u), 0.0, float(w), 0.0, float(aoa), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt_i])
            u_0 = array([0.0, float(de), 0.0, 1])
            beta = arctan(crosswind / v)

            # stability and control
            omega, zeta_sp[i_alt, i_mach], cap[i_alt, i_mach] = short_period_mode(plane, x_0, u_0)
            omega_dr[i_alt, i_mach], zeta_dr[i_alt, i_mach] = dutch_roll_mode(plane, x_0, u_0)
            t_2_d_sp[i_alt, i_mach] = spiral_mode(plane, x_0, u_0)
            t_roll[i_alt, i_mach] = roll_mode(plane, x_0, u_0)
            sm[i_alt, i_mach] = static_margin_nonlinear(plane, mach_i, alt_i, aoa, de)
            c_latdir = latdir_stability_nonlinear(plane, mach_i, alt_i, aoa, de)
            c_n[i_alt, i_mach] = c_latdir[1]
            c_r[i_alt, i_mach] = c_latdir[0]
            out_beta = trim_aileron_rudder_nonlinear(plane, v, alt_i, float(aoa), beta, 0, 0)
            dr_beta[i_alt, i_mach] = out_beta[1]
            da_beta[i_alt, i_mach] = out_beta[0]
            out_p = trim_aileron_nonlinear(plane, v, alt_i, p)
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
    plot_or_save(plt, show_plot, save_plot, name, 'balanced_field_length')

    # plotting
    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, sm[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Static Margin')
    plt.grid(True)
    plt.legend()
    plt.ylabel('SM [%MAC]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'static_margin')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, c_n[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Directional Stability')
    plt.grid(True)
    plt.legend()
    plt.ylabel('CNBeta [1/rad]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'directional_stability')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, c_r[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('lateral Stability')
    plt.grid(True)
    plt.legend()
    plt.ylabel('CLBeta [1/rad]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'lateral_stability')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(omega_dr[i_alt, :], zeta_dr[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
        plt.ylim((0, 1))
    plt.title("Dutch-Roll Damping")
    plt.grid(True)
    plt.legend()
    plot_dr()
    plot_or_save(plt, show_plot, save_plot, name, 'dutch_roll')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(zeta_sp[i_alt, :], cap[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
        plt.ylim((0, 1))
    plt.title("Short-Period Damping")
    plt.grid(True)
    plt.legend()
    plot_sp()
    plot_or_save(plt, show_plot, save_plot, name, 'short_period')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, t_roll[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Roll Mode')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Roll Time Constant [sec]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'roll_mode')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, t_2_d_sp[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Spiral Mode')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Time To Double [sec]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'spiral_mode')

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
        plot_or_save(plt, show_plot, save_plot, name, 'maneuver_capability_%d' % (altitudes[i_alt]))

    plt.figure()
    plt.subplot(2, 1, 1)
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, dr_beta[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Sideslip')
    plt.grid(True)
    plt.legend()
    plt.ylabel('rudder [deg]')
    plt.subplot(2, 1, 2)
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, da_beta[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.grid(True)
    plt.legend()
    plt.ylabel('aileron [deg]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'sideslip_capability')

    plt.figure()
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, da_roll[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Roll Control')
    plt.grid(True)
    plt.legend()
    plt.ylabel('aileron [deg]')
    plt.xlabel('Mach')
    plot_or_save(plt, show_plot, save_plot, name, 'roll_control')

    plt.figure()
    levels = linspace(0, 100000, 101)
    cs = plt.contour(machs, altitudes, p_s, levels)
    plt.clabel(cs, levels, fmt='%1.0f')
    plt.title('Specific Excess Power (fpm)')
    plt.grid(True)
    plt.xlabel('Mach')
    plt.ylabel('Altitude [ft]')
    plot_or_save(plt, show_plot, save_plot, name, 'specific_excess_power')

    plt.figure()
    levels = linspace(0, 5000, 51)
    cs = plt.contour(machs, altitudes, r, levels)
    plt.clabel(cs, levels, fmt='%1.0f')
    plt.title('range [nm]')
    plt.grid(True)
    plt.xlabel('Mach')
    plt.ylabel('Altitude [ft]')
    plot_or_save(plt, show_plot, save_plot, name, 'range')
