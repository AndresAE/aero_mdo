"""Returns force and moment coefficients for fuselages."""
from numpy import array, ceil, pi
from scipy.optimize import minimize
from src.common import unit_conversions
from src.modeling.aerodynamics import friction_coefficient, pressure_drag


class Fuselage:
    def __init__(self, aircraft):
        self.plane = aircraft

    def cross_section(self, tol=10e-3):
        """return far 14.25 compliant fuselage cross section parameters."""
        fus = self.plane['fuselage']
        w = fus['seats_row'] * unit_conversions.far_25_seat_width() + unit_conversions.far_25_aisle_width()
        ha = unit_conversions.far_25_aisle_height()
        hs = unit_conversions.far_25_head_room()

        def major_axis(x):
            ma = ha - x[0]
            return ma

        def obj(x):
            a_obj = major_axis(x)
            b_obj = x[1]
            cir = 2 * pi * ((a_obj + b_obj) / 2) ** 0.5
            return cir

        def cabin_constraint(x):
            a_c = major_axis(x)
            b_c = x[1]
            x_1 = w / 2
            y_1 = -x[0]
            # x_2 = x_1
            # y_2 = hs - x[0]
            out_1 = ((x_1 / b_c) ** 2) + ((y_1 / a_c) ** 2)
            # out_2 = ((x_2 / b_c) ** 2) + ((y_2 / a_c) ** 2)
            return out_1 - 1

        lim = ([0.01, ha], [1, w * 2])
        x0 = array([1, 1])
        u_out = minimize(obj, x0, bounds=lim, tol=tol,
                         constraints=({'type': 'eq', 'fun': cabin_constraint}),
                         options=({'maxiter': 200}))
        c = u_out['x']
        a = major_axis(c)
        b = c[1]
        dy = c[0]
        return a, b, dy, w

    def far_25_length(self):
        """return far 14.25 compliant fuselage length."""
        fus = self.plane['fuselage']
        n_row = ceil(fus['pax'] / fus['seats_row'])
        l_row = n_row * unit_conversions.far_25_seat_pitch()
        n_exit_l = ceil(n_row / 60)
        if fus['pax'] < 20:
            n_exits = max([1, n_exit_l])
            l_exits = n_exits * unit_conversions.type_3_exit_width()
        elif fus['pax'] < 40:
            n_exits = max([2, n_exit_l])
            l_exits = n_exits * unit_conversions.type_2_exit_width()
        else:
            n_exits = max([2, n_exit_l])
            l_exits = n_exits * unit_conversions.type_1_exit_width()
        l_fus = l_exits + l_row
        return l_fus

    def parasite_drag_fuselage(self, mach, altitude):
        """return parasitic drag coefficient of the fuselage."""
        fuselage = self.plane['fuselage']
        s_w = self.plane['wing']['planform']
        s_wet_s = 2 * (fuselage['length'] * fuselage['width']
                       + fuselage['length'] * fuselage['height']
                       + fuselage['height'] * fuselage['width']) / s_w  # []
        c_f = friction_coefficient(mach, altitude, fuselage['length'])  # []
        c_d_p = pressure_drag((fuselage['width'] + fuselage['height']) / (2 * fuselage['length']))
        c_d_0 = c_d_p * c_f * s_wet_s  # []
        return c_d_0
