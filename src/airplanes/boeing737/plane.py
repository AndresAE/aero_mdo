"""airplane geometry file."""
plane = {
    'name': 'boeing737',
    'wing': {
        'type': 'wing',
        'planform': 980,  # [ft^2]
        'aspect_ratio': 12.9,  # []
        'sweep_LE': 27,  # [deg]
        'taper': 0.15,  # []
        'station': 39,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 0,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 5,  # [deg]
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
        'planform': 373.7,  # [ft^2]
        'aspect_ratio': 5.9,  # []
        'sweep_LE': 35,  # [deg]
        'taper': 0.39,  # []
        'station': 88.3,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 8.1,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 8,  # [deg]
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
        'planform': 263.1,  # [ft^2]
        'aspect_ratio': 4.86,  # []
        'sweep_LE': 36,  # [deg]
        'taper': 0.21,  # []
        'station': 79.1,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 8.1,  # [ft]
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
        'length': 97.8,  # [ft]
        'width': 13.1,  # [ft]
        'height': 13.1,  # [ft]
        'l_cabin': 60,  # [ft]
        'l_cockpit': 13,  # [ft]
        'pax': 123,
        'lavatories': 2,
        'seats_row': 6,
    },
    'weight': {
        'weight': 144500,  # [lb]
        'inertia': [[441933, 0, 120000], [0, 821088, 0], [120000, 0, 1218803]],  # [slug*ft^2]
        'cg': [60, 0, 4]  # [ft]
    },
    'propulsion': {
        'n_engines': 2,
        'const_mass': 0,
        'fuel_mass': 46750 / 32.2,  # [slug]
        'energy_density': 11.8 * 2655224 / 0.0685218,  # [ft^2 / s^2]
        'total_efficiency': 0.4,
        'engine_1': {
            'type': 'jet',
            'station': 36,  # [ft]
            'waterline': 4,  # [ft]
            'buttline': -15,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'thrust': 20000,  # [lb]
        },
        'engine_2': {
            'type': 'jet',
            'station': 36,  # [ft]
            'waterline': 4,  # [ft]
            'buttline': 15,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'thrust': 20000,  # [lb]
        },
    },
    'landing_gear': {
        'nose': [15, 0, -4],  # [ft]
        'nose_diameter': 1,  # [ft]
        'nose_width': 0.5,  # [ft]
        'main': [65, 8, -8],  # [ft], left (mirrored on the right)
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
        'cruise_mach': 0.785,  # [ft/s]
        'cruise_altitude': 25000,  # [ft]
        'range': 3235,  # [nm]
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
        'mach': [0.2, 0.9],
        'alpha': [-5, 15],  # [deg]
        'beta': [-10, 10],  # [deg]
        'p': [-20, 20],  # [deg/s]
        'q': [-20, 20],  # [deg/s]
        'r': [-20, 20],  # [deg/s]
        'altitude': [0, 45000]  # [ft]
    }
}
