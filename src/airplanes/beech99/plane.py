"""airplane geometry file."""
plane = {
    'name': 'beech99',
    'type': 'transport',
    'wing': {
        'type': 'wing',
        'planform': 280,  # [ft^2]
        'aspect_ratio': 7.55,  # []
        'sweep_LE': 3,  # [deg]
        'taper': 0.46,  # []
        'station': 12.5,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 0,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 7,  # [deg]
        'airfoil': '2412',  # [naca airfoils]
        'alpha_stall': 12,  # [deg]
        'n_controls': 2,  # []
        'control_1': {
            'name': 'aileron_l',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.4,
            'b_1': -0.94,
            'b_2': -0.75,
        },
        'control_2': {
            'name': 'flap_l',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.4,
            'b_1': -0.75,
            'b_2': -0.94,
        },
    },
    'horizontal': {
        'type': 'wing',
        'planform': 98.44,  # [ft^2]
        'aspect_ratio': 5.14,  # []
        'sweep_LE': 21,  # [deg]
        'taper': 0.55,  # []
        'station': 38,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 4,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 7,  # [deg]
        'airfoil': '0012',  # [naca airfoils]
        'n_controls': 2,
        'control_1': {
            'name': 'elevator_l',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.25,
            'b_1': -1,
            'b_2': 0.1,
        },
        'control_2': {
            'name': 'elevator_r',
            'limits': [-30, 30],  # [deg]
            'cf_c': 0.25,
            'b_1': 0.1,
            'b_2': 1,
        },
    },
    'vertical': {
        'type': 'vertical',
        'planform': 38.9,  # [ft^2]
        'aspect_ratio': 4.48,  # []
        'sweep_LE': 47,  # [deg]
        'taper': 0.44,  # []
        'station': 35.8,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 5.3,  # [ft]
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
        'length': 44,  # [ft]
        'width': 7,  # [ft]
        'height': 6,  # [ft]
        'l_cabin': 18,  # [ft]
        'l_cockpit': 5,  # [ft]
        'pax': 15,
        'lavatories': 1,
        'seats_row': 2,
    },
    'weight': {
        'weight': 10400,  # [lb]
        'inertia': [[441933, 0, 120000], [0, 821088, 0], [120000, 0, 1218803]],  # [slug*ft^2]
        'cg': [18, 0, 4]  # [ft]
    },
    'propulsion': {
        'n_engines': 2,
        'const_mass': 0,
        'fuel_mass': 2244 / 32.2,  # [slug]
        'energy_density': 11.8 * 2655224 / 0.0685218,  # [ft^2 / s^2]
        'total_efficiency': 0.4,
        'engine_1': {
            'type': 'prop',
            'station': 11,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': -5.5,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 7.5,  # [ft]
            'rpm_max': 2000,  # [rpm]
            'pitch': 20  # [deg]
        },
        'engine_2': {
            'type': 'prop',
            'station': 11,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': 5.5,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 7.5,  # [ft]
            'rpm_max': 2000,  # [rpm]
            'pitch': 20  # [deg]
        },
    },
    'landing_gear': {
        'nose': [2, 0, -3],  # [ft]
        'nose_diameter': 0.75,  # [ft]
        'nose_width': 0.4,  # [ft]
        'main': [19, 5.5, -3],  # [ft], left (mirrored on the right)
        'main_diameter': 1,  # [ft]
        'main_width': 0.5,  # [ft]
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
        'bfl': 2000,  # [ft]
        'stall_speed': 50,  # [ft/s]
        'to_altitude': 5000,  # [ft]
        'cruise_mach': 0.35,  # [ft/s]
        'cruise_altitude': 1000,  # [ft]
        'range': 910,  # [nm]
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
        'mach': [0.1, 0.4],
        'alpha': [-5, 15],  # [deg]
        'beta': [-10, 10],  # [deg]
        'p': [-20, 20],  # [deg/s]
        'q': [-20, 20],  # [deg/s]
        'r': [-20, 20],  # [deg/s]
        'altitude': [0, 15000]  # [ft]
    }
}
