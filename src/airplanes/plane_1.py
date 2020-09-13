"""airplane geometry file."""
plane = {
    'wing': {
        'type': 'wing',
        'planform': 650,  # [ft^2]
        'aspect_ratio': 8,  # []
        'sweep_LE': 5,  # [deg]
        'taper': 0.5,  # []
        'dihedral': 0,  # [deg]
        'station': 37.5,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 1,  # [ft]
        'incidence': 0,  # [deg]
        'c_l_alpha': 5,  # [1/rad]
        'alpha_zero_lift': -1,  # [deg]
        'airfoil_thickness': 0.15,  # []
        'airfoil_cm0': 0,  # []
        'alpha_stall': 12,  # [deg]
    },
    'horizontal': {
        'type': 'wing',
        'planform': 100,  # [ft^2]
        'aspect_ratio': 4,  # []
        'sweep_LE': 10,  # [deg]
        'taper': 0.465,  # []
        'dihedral': 0,  # [deg]
        'station': 85,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 7,  # [ft]
        'incidence': 0,  # [deg]
        'c_l_alpha': 5,  # [1/rad]
        'alpha_zero_lift': 0,  # [deg]
        'airfoil_thickness': 0.1,  # []
        'airfoil_cm0': 0  # []
    },
    'vertical': {
        'type': 'vertical',
        'planform': 100,  # [ft^2]
        'aspect_ratio': 2,  # []
        'sweep_LE': 20,  # [deg]
        'taper': 0.5,  # []
        'station': 78,  # [ft]
        'buttline': 0,  # [ft]
        'waterline': 8,  # [ft]
        'c_l_alpha': 5,  # [1/rad]
        'alpha_zero_lift': 0,  # [deg]
        'airfoil_thickness': 0.15  # []
    },
    'elevator': {
        'type': 'control_surface',
        'parent': 'horizontal',
        'limits': [-15, 15],  # [deg]
        'chord_ratio': 0.37,  # []
        'gain': -1
    },
    'aileron_l': {
        'type': 'control_surface',
        'parent': 'wing',
        'chord_ratio': 0.3,
        'x_2_b': 0.98,
        'x_1_b': 0.75,
        'limits': [-15, 15],  # [deg]
        'gain': 0.5
    },
    'aileron_r': {
        'type': 'control_surface',
        'parent': 'wing',
        'chord_ratio': 0.35,
        'x_2_b': 0.98,
        'x_1_b': 0.75,
        'limits': [-15, 15],  # [deg]
        'gain': 0.5
    },
    'rudder': {
        'type': 'control_surface',
        'parent': 'vertical',
        'limits': [-15, 15],  # [deg]
        'chord_ratio': 0.35,  # []
        'span_ratio': 1  # []
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
        'weight': 41316.6,  # [lb]
        'inertia': [[160937, 0, 30000], [0, 234038, 0], [30000, 0, 348056]],  # [slug*ft^2]
        'cg': [43, 0, 2]  # [ft]
    },
    'propulsion': {
        'n_engines': 6,
        'type': 'electric',
        'battery_weight': 12000,  # [lb]
        'efficiency': 0.75,
        'engine_1': {
            'type': 'prop',
            'station': 36,  # [ft]
            'waterline': 1,  # [ft]
            'buttline': -40,  # [ft]
            'thrust_angle': 0,  # [deg]
            'toe_angle': 0,  # [deg]
            'diameter': 4,  # [ft]
            'rpm_max': 10000,  # [rpm]
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
            'rpm_max': 10000,  # [rpm]
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
            'rpm_max': 10000,  # [rpm]
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
            'rpm_max': 10000,  # [rpm]
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
            'rpm_max': 10000,  # [rpm]
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
            'rpm_max': 10000,  # [rpm]
            'pitch': 20  # [deg]
        },
    },
    'landing_gear': {
        'nose': [15, 0, -4],  # [ft]
        'nose_diameter': 1,  # [ft]
        'nose_width': 0.5,  # [ft]
        'main': [49, 12, -4],  # [ft]
        'main_diameter': 1.5,  # [ft]
        'main_width': 1.5,  # [ft]
        'mu_roll': 0.02,
        'mu_brake': 0.1,
        'c_d': 0.01
    },
    'control_law': {
        'pitch': {
            'damping_gain': 0,
            'stiffness_gain': 0
        },
        'roll': {
            'damping_gain': -0.5,
            'stiffness_gain': 0
        },
        'yaw': {
            'damping_gain': 0.5,
            'stiffness_gain': -0.005
        }
    }
}
