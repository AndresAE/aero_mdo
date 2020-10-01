"""Returns force and moment coefficients for lifting surfaces."""
from numpy import sqrt, cos, deg2rad, pi
from src.modeling.aerodynamics import polhamus, friction_coefficient, pressure_drag
from src.modeling.trapezoidal_wing import root_chord, mac, span, sweep_x, x_mac, y_mac


class LiftingSurface:
    def __init__(self, wing):
        self.wing = wing

    def c_l_alpha_wing(self, mach):
        """return lifting surface lift curve slope."""
        c_l_alpha = 2 * pi * 0.9
        ar = self.wing['aspect_ratio']  # []
        sweep_le = self.wing['sweep_LE']  # [deg]
        taper = self.wing['taper']  # []
        c_l_alpha_3d = polhamus(c_l_alpha, ar, mach, taper, sweep_le)   # [1/rad]
        return c_l_alpha_3d

    def d_epsilon_d_alpha(self, ht, mach):
        """return downwash gradient wrt angle of attack."""
        ar = self.wing['aspect_ratio']  # []
        taper = self.wing['taper']  # []
        root_chord_wing = root_chord(ar, self.wing['planform'], taper)  # [ft]
        root_chord_ht = root_chord(ht['aspect_ratio'], ht['planform'], ht['taper'])  # [ft]
        x_wh = (ht['station'] + root_chord_ht / 4) - (self.wing['station'] + root_chord_wing / 4)  # [ft]
        z_wh = ht['waterline'] - self.wing['waterline']  # [ft]
        b = span(ar, self.wing['planform'])  # [ft]
        r = 2 * x_wh / b  # []
        m = 2 * z_wh / b  # []
        k_ar = (1 / ar) - 1 / (1 + ar ** 1.7)  # []
        k_taper = (10 - 3 * taper) / 7  # []
        k_mr = (1 - (m / 2)) / (r ** 0.333)  # []
        sweep_25 = sweep_x(ar, taper, self.wing['sweep_LE'], 0.25)  # [deg]
        beta = sqrt(1 - mach ** 2)  # []
        de_da = 4.44 * beta * (k_ar * k_taper * k_mr * sqrt(cos(deg2rad(sweep_25)))) ** 1.19  # []
        return de_da

    def aerodynamic_center(self):
        """return lifting surface aerodynamic center."""
        c_bar = mac(self.wing['aspect_ratio'], self.wing['planform'], self.wing['taper'])  # [ft]
        y = y_mac(self.wing['aspect_ratio'], self.wing['planform'], self.wing['taper'])  # [ft]
        x = x_mac(y, self.wing['sweep_LE'])  # [ft]
        x_ac = self.wing['station'] + x + c_bar / 4  # [ft]
        return x_ac

    def parasite_drag(self, mach, altitude):
        """return parasitic drag coefficient of the lifting surface."""
        t_c = self.wing['airfoil']
        t_c = int(t_c[-2:]) / 100
        s_wet_s = (2 + 2 * (t_c / self.wing['aspect_ratio']) +
                   2 * t_c)  # []
        c_bar = mac(self.wing['aspect_ratio'], self.wing['planform'], self.wing['taper'])  # [ft]
        c_f = friction_coefficient(mach, altitude, c_bar)  # []
        c_d_p = pressure_drag(t_c)
        c_d_0 = c_d_p * c_f * s_wet_s  # []
        return c_d_0

    def d_sigma_d_beta_ht(self, ht, fuselage):
        """return sidewash gradient wrt sideslip."""
        wing = self.wing
        ar = wing['aspect_ratio']
        s_w = wing['planform']
        s_h = ht['planform']
        z_w = wing['waterline']
        d = fuselage['width']
        sweep_4 = sweep_x(ar, wing['taper'], wing['sweep_LE'], 0.25)
        eta_ds_db = 0.724 + 3.06 * (s_h / s_w) / (1 + cos(deg2rad(sweep_4))) + 0.4 * z_w / d + 0.009 * ar
        return eta_ds_db

    def d_sigma_d_beta_vt(self, vt, fuselage):
        """return sidewash gradient wrt sideslip."""
        wing = self.wing
        ar = wing['aspect_ratio']
        s_w = wing['planform']
        s_v = vt['planform']
        z_w = wing['waterline']
        d = fuselage['height']
        sweep_4 = sweep_x(ar, wing['taper'], wing['sweep_LE'], 0.25)
        eta_ds_db = 0.724 + 3.06 * (s_v / s_w) / (1 + cos(deg2rad(sweep_4))) + 0.4 * z_w / d + 0.009 * ar
        return eta_ds_db
