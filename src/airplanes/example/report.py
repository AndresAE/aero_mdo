from matplotlib import pyplot as plt
from numpy import array, cos, deg2rad, pi, sin, zeros
from src.common import Earth
from src.analysis.longitudinal import balanced_field_length, maneuvering, short_period_mode, plot_sp, static_margin, \
    specific_excess_power
from src.analysis.lateral_directional import directional_stability, lateral_stability, dutch_roll_mode, plot_dr, \
    roll_mode, spiral_mode
from src.analysis.trim import trim_alpha_de
from src.common import Atmosphere
g = Earth(0).gravity()  # f/s2

# Requirements
n_z = [-1, 1, 2.5]  # [g]


# Sweep
def report_sweep(plane, machs, altitudes):
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

            # stability and control
            omega, zeta_sp[i_alt, i_mach], cap[i_alt, i_mach] = short_period_mode(plane, x_0, u_0)
            omega_dr[i_alt, i_mach], zeta_dr[i_alt, i_mach] = dutch_roll_mode(plane, x_0, u_0)
            t_2_d_sp[i_alt, i_mach] = spiral_mode(plane, x_0, u_0)
            t_roll[i_alt, i_mach] = roll_mode(plane, x_0, u_0)
            sm[i_alt, i_mach] = static_margin(plane, mach_i)
            c_n[i_alt, i_mach] = directional_stability(plane, mach_i, aoa)
            c_r[i_alt, i_mach] = lateral_stability(plane, mach_i, aoa)

            # performance
            p_s[i_alt, i_mach] = specific_excess_power(plane, x_0, u_0)
            alpha_nz[i_alt, i_mach, :], de_nz[i_alt, i_mach, :] = maneuvering(plane, mach_i, alt_i, n_z)

            i_mach = i_mach + 1
        i_alt = i_alt + 1

    x_0 = array([float(0.01), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
    u_0 = array([0.0, -20 * pi / 180, 0.0, 1])
    balanced_field_length(plane, x_0, u_0)

    # plotting
    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, sm[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Static Margin')
    plt.grid(True)
    plt.legend()
    plt.ylabel('SM [%MAC]')
    plt.xlabel('Mach')
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, c_n[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Directional Stability')
    plt.grid(True)
    plt.legend()
    plt.ylabel('CNBeta [1/rad]')
    plt.xlabel('Mach')
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, c_r[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('lateral Stability')
    plt.grid(True)
    plt.legend()
    plt.ylabel('CLBeta [1/rad]')
    plt.xlabel('Mach')
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.plot(omega_dr[i_alt, :], zeta_dr[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
        plt.ylim((0, 1))
    plt.title("Dutch-Roll Damping")
    plt.grid(True)
    plt.legend()
    plot_dr()
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.plot(zeta_sp[i_alt, :], cap[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
        plt.ylim((0, 1))
    plt.title("Short-Period Damping")
    plt.grid(True)
    plt.legend()
    plot_sp()
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, t_roll[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Roll Mode')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Roll Time Constant [sec]')
    plt.xlabel('Mach')
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.plot(machs, t_2_d_sp[i_alt, :], label="Alt %d ft" % (altitudes[i_alt]))
    plt.title('Spiral Mode')
    plt.grid(True)
    plt.legend()
    plt.ylabel('Time To Double [sec]')
    plt.xlabel('Mach')
    plt.show()

    for i_alt in range(0, len(altitudes)):
        plt.figure(figsize=(12, 6))
        for ni in range(0, len(n_z)):
            plt.subplot(1, 2, 1)
            plt.plot(machs, de_nz[i_alt, :, ni], label="Nz %d g" % (n_z[ni]))
            plt.subplot(1, 2, 2)
            plt.plot(machs, alpha_nz[i_alt, :, ni], label="Nz %d g" % (n_z[ni]))
        plt.subplot(1, 2, 1)
        plt.grid(True)
        plt.title("Maneuver Capability, %d ft" % (altitudes[i_alt]))
        plt.xlabel('Mach')
        plt.ylabel('Elevator Deflection [deg]')
        plt.legend()
        plt.subplot(1, 2, 2)
        plt.grid(True)
        plt.xlabel('Mach')
        plt.ylabel('Angle of Attack [deg]')
        plt.legend()
        plt.show()

    plt.figure()
    levels = [0, 100, 500, 1000]
    cs = plt.contour(machs, altitudes, p_s, levels)
    plt.clabel(cs, levels, fmt='%1.0f')
    plt.title('Specific Excess Power (fpm)')
    plt.grid(True)
    plt.xlabel('Mach')
    plt.ylabel('Altitude [ft]')
    plt.show()
