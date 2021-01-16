"""airplane geometry file."""
plane = {
    'name': 'example',
    'type': 'transport',
    'wing': {
        'type': 'wing',
        'planform': 449.19,  # [ft^2]
        'aspect_ratio': 12,  # []
        'sweep_LE': 0,  # [deg]
        'taper': 0.5,  # []
        'station': 26.3,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 7.9,  # [ft]
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
        'planform': 39.2,  # [ft^2]
        'aspect_ratio': 3,  # []
        'sweep_LE': 30,  # [deg]
        'taper': 0.465,  # []
        'station': 60.3,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 5.9,  # [ft]
        'incidence': 0,  # [deg]
        'dihedral': 0,  # [deg]
        'airfoil': '0012',  # [naca airfoils]
        'n_controls': 2,
        'control_1': {
            'name': 'elevator_l',
            'limits': [-25, 15],  # [deg]
            'cf_c': 1,
            'b_1': -1,
            'b_2': 0.1,
        },
        'control_2': {
            'name': 'elevator_r',
            'limits': [-25, 15],  # [deg]
            'cf_c': 1,
            'b_1': 0.1,
            'b_2': 1,
        },
    },
    'vertical': {
        'type': 'vertical',
        'planform': 18.9,  # [ft^2]
        'aspect_ratio': 3,  # []
        'sweep_LE': 30,  # [deg]
        'taper': 0.5,  # []
        'station': 58.3,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 5.91,  # [ft]
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
        'length': 63.3,  # [ft]
        'width': 5.95,  # [ft]
        'height': 7.91,  # [ft]
        'l_cabin': 30,  # [ft]
        'l_cockpit': 5,  # [ft]
        'pax': 30,
        'lavatories': 0,
        'seats_row': 2,
    },
    'weight': {
        'weight': 15309,  # [lb]
        'inertia': [[57700, 0, 19233], [0, 42930, 0], [19233, 0, 88965]],  # [slug*ft^2]
        'cg': [28.9, 0, 5.6]  # [ft]
    },
    'propulsion': {
        'n_engines': 2,
        'const_mass': 1,
        'fuel_mass': 7.2 / 32.2,  # [slug]
        'energy_density': 33.3 * 2655224 / 0.0685218,  # [ft^2 / s^2]
        'total_efficiency': 0.7,
        'engine_1': {
            'type': 'prop',
            'station': 26.9,  # [ft]
            'waterline': 6,  # [ft]
            'buttline': -12,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4.75,  # [ft]
            'rpm_max': 6651,  # [rpm]
            'pitch': 21.2  # [deg]
        },
        'engine_2': {
            'type': 'prop',
            'station': 26.9,  # [ft]
            'waterline': 6,  # [ft]
            'buttline': 12,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4.75,  # [ft]
            'rpm_max': 6651,  # [rpm]
            'pitch': 21.2  # [deg]
        },
    },
    'landing_gear': {
        'nose': [6.95, 0, -1.6],  # [ft]
        'nose_diameter': 1,  # [ft]
        'nose_width': 0.5,  # [ft]
        'main': [30.65, 3, -1.6],  # [ft], left (mirrored on the right)
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
        'stall_speed': 120,  # [ft/s]
        'to_altitude': 5000,  # [ft]
        'cruise_mach': 0.42,  # [ft/s]
        'cruise_altitude': 22000,  # [ft]
        'range': 825,  # [nm]
    },
    'stability_and_control': {
        'c_n_b': 0.12,  # [1/deg]
        'sm': 0.03,  # [% mac]
        'zeta_dr': 0.6,
        'zeta_sp': 0.4,
        'roll_rate': 30 / 1.5,  # [seconds] to 30 deg of roll
        'rotation_margin': 1,  # [deg]
        'crosswind': 15,  # [ft/s]
        'lto_roll_angle': 5,  # [deg]
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
