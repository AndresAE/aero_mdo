"""airplane geometry file."""
plane = {
    'name': 'example',
    'wing': {
        'type': 'wing',
        'planform': 640,  # [ft^2]
        'aspect_ratio': 8,  # []
        'sweep_LE': 0,  # [deg]
        'taper': 0.5,  # []
        'station': 37.5,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 1,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 0,  # [deg]
        'airfoil': '4412',  # [naca airfoils]
        'alpha_stall': 12,  # [deg]
        'n_controls': 4,  # []
        'control_1': {
            'name': 'aileron_l',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.25,
            'b_1': -1,
            'b_2': -0.75,
        },
        'control_2': {
            'name': 'flap_l',
            'limits': [0, 15],  # [deg]
            'cf_c': 0.2,
            'b_1': -0.7,
            'b_2': -0.2,
        },
        'control_3': {
            'name': 'flap_r',
            'limits': [0, 15],  # [deg]
            'cf_c': 0.2,
            'b_1': 0.2,
            'b_2': 0.7,
        },
        'control_4': {
            'name': 'aileron_r',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.25,
            'b_1': 0.75,
            'b_2': 1,
        },
    },
    'horizontal': {
        'type': 'wing',
        'planform': 50.5,  # [ft^2]
        'aspect_ratio': 5,  # []
        'sweep_LE': 10,  # [deg]
        'taper': 0.465,  # []
        'station': 85,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 5,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 0,  # [deg]
        'airfoil': '0012',  # [naca airfoils]
        'n_controls': 2,
        'control_1': {
            'name': 'elevator_l',
            'limits': [-25, 15],  # [deg]
            'cf_c': 0.3,
            'b_1': -1,
            'b_2': 0.1,
        },
        'control_2': {
            'name': 'elevator_r',
            'limits': [-25, 15],  # [deg]
            'cf_c': 0.3,
            'b_1': 0.1,
            'b_2': 1,
        },
    },
    'vertical': {
        'type': 'vertical',
        'planform': 39.2,  # [ft^2]
        'aspect_ratio': 4,  # []
        'sweep_LE': 20,  # [deg]
        'taper': 0.5,  # []
        'station': 78,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 5,  # [ft]
        'airfoil': '0012',  # [naca airfoils]
        'n_controls': 1,
        'control_1': {
            'name': 'rudder',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.3,
            'b_1': 0.1,
            'b_2': 1,
        },
    },
    'fuselage': {
        'length': 50,  # [ft]
        'width': 7,  # [ft]
        'height': 7,  # [ft]
        'l_cabin': 30,  # [ft]
        'l_cockpit': 5,  # [ft]
        'pax': 20,
        'lavatories': 0,
        'seats_row': 2,
    },
    'weight': {
        'weight': 24000,  # [lb]
        'inertia': [[141933, 0, 30000], [0, 221088, 0], [30000, 0, 318803]],  # [slug*ft^2]
        'cg': [40, 0, 5]  # [ft]
    },
    'propulsion': {
        'n_engines': 6,
        'const_mass': 1,
        'fuel_mass': 6000 / 32.2,  # [slug]
        'energy_density': 12.2 * 2655224 / 0.0685218,  # [ft^2 / s^2]
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
        'stall_speed': 100,  # [ft/s]
        'to_altitude': 5000,  # [ft]
        'cruise_mach': 0.5,  # [ft/s]
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
    },
    'flight_envelope': {
        'mach': [0.2, 0.6],
        'alpha': [-5, 15],  # [deg]
        'beta': [-10, 10],  # [deg]
        'p': [-20, 20],  # [deg/s]
        'q': [-20, 20],  # [deg/s]
        'r': [-20, 20],  # [deg/s]
        'altitude': [0, 30000]  # [ft]
    }
}
