"""airplane geometry file."""
plane = {
    'wing': {
        'type': 'wing',
        'planform': 700,  # [ft^2]
        'aspect_ratio': 8,  # []
        'sweep_LE': 0,  # [deg]
        'taper': 0.5,  # []
        'station': 37.5,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 1,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 0,  # [deg]
        'airfoils': ['4412', '4412'],  # [naca airfoils]
        'alpha_stall': 12,  # [deg]
        'n_controls': 4,  # []
        'control_1': {
            'name': 'aileron_l',
            'cf_c': 0.25,
            'b_1': -1,
            'b_2': -0.75,
        },
        'control_2': {
            'name': 'flap_l',
            'cf_c': 0.2,
            'b_1': -0.7,
            'b_2': -0.2,
        },
        'control_3': {
            'name': 'flap_r',
            'cf_c': 0.2,
            'b_1': 0.2,
            'b_2': 0.7,
        },
        'control_4': {
            'name': 'aileron_r',
            'cf_c': 0.25,
            'b_1': 0.75,
            'b_2': 1,
        },
    },
    'horizontal': {
        'type': 'wing',
        'planform': 100,  # [ft^2]
        'aspect_ratio': 4,  # []
        'sweep_LE': 10,  # [deg]
        'taper': 0.465,  # []
        'station': 85,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 7,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 0,  # [deg]
        'airfoils': ['0012', '0012'],  # [naca airfoils]
        'n_controls': 2,
        'control_1': {
            'name': 'elevator_l',
            'limits': [-25, 15],  # [deg]
            'cf_c': 0.3,
            'b_1': -0.999,
            'b_2': -0.01,
        },
        'control_2': {
            'name': 'elevator_r',
            'limits': [-25, 15],  # [deg]
            'cf_c': 0.3,
            'b_1': 0.01,
            'b_2': 0.999,
        },
    },
    'vertical': {
        'type': 'vertical',
        'planform': 80,  # [ft^2]
        'aspect_ratio': 2,  # []
        'sweep_LE': 20,  # [deg]
        'taper': 0.5,  # []
        'station': 78,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 8,  # [ft]
        'airfoils': ['0012', '0012'],  # [naca airfoils]
        'n_controls': 1,
        'control_1': {
            'name': 'rudder',
            'cf_c': 0.3,
            'b_1': 0,
            'b_2': 1,
        },
    },
    'fuselage': {
        'length': 90,  # [ft]
        'width': 10,  # [ft]
        'height': 10,  # [ft]
        'l_cabin': 60,  # [ft]
        'l_cockpit': 8,  # [ft]
        'theta_cockpit': 45,  # [deg]
        'theta_tailstrike': 12,  # [deg]
        'pax': 20
    },
    'weight': {
        'weight': 32000,  # [lb]
        'inertia': [[141933, 0, 30000], [0, 221088, 0], [30000, 0, 318803]],  # [slug*ft^2]
        'cg': [42, 0, 1]  # [ft]
    },
    'propulsion': {
        'n_engines': 6,
        'const_mass': 1,
        'fuel_mass': 6000 / 32.2,  # [slug]
        'energy_density': 11.8 * 2655224 / 0.0685218,  # [ft^2 / s^2]
        'total_efficiency': 0.4,
        'engine_1': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': -40,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 11000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_2': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': -30,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 11000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_3': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': -20,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 11000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_4': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': 20,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 11000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_5': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': 30,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 11000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_6': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': 40,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 11000,  # [rpm]
            'pitch': 20  # [deg]
        },
    },
    'landing_gear': {
        'nose': [15, 0, -4],  # [ft]
        'nose_diameter': 1,  # [ft]
        'nose_width': 0.5,  # [ft]
        'main': [46, 12, -4],  # [ft], left (mirrored on the right)
        'main_diameter': 1.5,  # [ft]
        'main_width': 1.5,  # [ft]
        'mu_roll': 0.02,
        'mu_brake': 0.1,
        'c_d': 0.02
    },
}
requirements = {
    'loads': {
        'n_z': [-1, 2.5],  # [g]
    },
    'performance': {
        'bfl': 5000,  # [ft]
        'to_altitude': 5000,  # [ft]
        'cruise_speed': 350,  # [ft/s]
        'cruise_altitude': 25000,  # [ft]
        'range': 500,  # [nm]
    },
    'stability_and_control': {
        'c_n_b': 0.05,  # [1/deg]
        'sm': 0.03,  # [% mac]
        'zeta_dr': 0.4,
        'zeta_sp': 0.4,
        'roll_rate': 30 / 1.5,  # [seconds] to 30 deg of roll
        'rotation_margin': 1,  # [deg]
        'crosswind': 15,  # [ft/s]
    }
}
