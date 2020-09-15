"""Returns force and moment coefficients for total aircraft."""
from numpy import deg2rad, interp, pi, sin, cos, tan
from src.modeling import Fuselage
from src.modeling import LiftingSurface
from src.modeling.trapezoidal_wing import mac, span, x_mac, y_mac
k_yv = 1


class Aircraft:
    def __init__(self, aircraft):
        self.plane = aircraft

    def c_l_zero(self, mach):
        """baseline lift coefficient."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        c_l_0_w = c_l_alpha_w * (deg2rad(wing['incidence'] - wing['alpha_zero_lift']))  # []
        epsilon = 2 * c_l_0_w / (pi * wing['aspect_ratio'])  # [rad]
        c_l_0_ht = c_l_alpha_ht * s_ht / s_w * (deg2rad(ht['incidence'] - ht['alpha_zero_lift'] - epsilon))  # []
        c_l_0 = c_l_0_w + c_l_0_ht  # []
        return c_l_0

    def c_l_alpha(self, mach):
        """returns lift curve slope for aircraft."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        downwash = LiftingSurface(wing).d_epsilon_d_alpha(ht, mach)  # []
        c_l_alpha_airplane = c_l_alpha_w + c_l_alpha_ht*s_ht/s_w*(1-downwash)  # [1/rad]
        return c_l_alpha_airplane

    def c_l_delta_elevator(self, mach):
        """returns lift curve of elevator wrt deflection angle."""
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        tau = self.plane['elevator']['chord_ratio']
        c_l_de = c_l_alpha_ht * s_ht / s_w * tau  # [1/rad]
        return c_l_de

    def c_l_pitch_rate(self, mach):
        """returns lift curve of elevator wrt pitch rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []
        c_l_q = 2 * c_l_alpha_ht * s_ht / s_w * (x_ac_ht_bar - cg_bar)
        return c_l_q

    def c_l_alpha_dot(self, mach):
        """returns lift curve of elevator wrt alpha rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []
        downwash = LiftingSurface(wing).d_epsilon_d_alpha(ht, mach)  # []
        c_l_adt = 2 * c_l_alpha_ht * s_ht / s_w * (x_ac_ht_bar - cg_bar) * downwash
        return c_l_adt

    def c_m_zero(self, mach, altitude):
        """baseline lift coefficient."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        vt = self.plane['vertical']
        s_vt = vt['planform']  # [ft^2]

        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        z_cg = self.plane['weight']['cg'][2]

        x_ac_w = LiftingSurface(wing).aerodynamic_center()  # [ft]
        x_ac_w_bar = x_ac_w / c_bar  # []

        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []

        c_l_0_w = c_l_alpha_w * (deg2rad(wing['incidence'] - wing['alpha_zero_lift']))  # []
        c_m_0_w = c_l_0_w * (cg_bar - x_ac_w_bar)
        epsilon = 2 * c_l_0_w / (pi * wing['aspect_ratio'])  # [rad]
        c_l_0_ht = c_l_alpha_ht * s_ht / s_w * (deg2rad(ht['incidence'] - ht['alpha_zero_lift'] - epsilon))  # []

        z_w = (z_cg - wing['waterline']) / c_bar
        z_ht = (z_cg - ht['waterline']) / c_bar
        z_vt = (z_cg - vt['waterline']) / c_bar
        z_f = (z_cg - self.plane['fuselage']['height'] / 2) / c_bar

        c_m_0_w_d = - LiftingSurface(wing).parasite_drag(mach, altitude) * z_w
        c_m_0_ht_d = - LiftingSurface(ht).parasite_drag(mach, altitude) * s_ht / s_w * z_ht
        c_m_0_vt_d = - LiftingSurface(vt).parasite_drag(mach, altitude) * s_vt / s_w * z_vt
        c_m_0_f_d = - Fuselage(self.plane).parasite_drag_fuselage(mach, altitude) * z_f
        c_m_0_ht = c_l_0_ht * (x_ac_ht_bar - cg_bar)
        c_m_0 = (wing['airfoil_cm0'] + ht['airfoil_cm0'] + c_m_0_w + c_m_0_ht
                 + c_m_0_w_d + c_m_0_ht_d + c_m_0_vt_d + c_m_0_f_d)  # []
        return c_m_0

    def c_m_alpha(self, mach):
        """returns pitching moment curve slope for aircraft."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        x_ac_w = LiftingSurface(wing).aerodynamic_center()  # [ft]
        x_ac_w_bar = x_ac_w / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        downwash = LiftingSurface(wing).d_epsilon_d_alpha(ht, mach)  # []
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []
        c_m_alpha_w = c_l_alpha_w * (cg_bar - x_ac_w_bar)  # [1/rad]
        c_m_alpha_ht = c_l_alpha_ht * s_ht / s_w * (1 - downwash) * (x_ac_ht_bar - cg_bar)  # [1/rad]
        c_m_alpha_airplane = c_m_alpha_w - c_m_alpha_ht  # [1/rad]
        return c_m_alpha_airplane

    def c_m_delta_elevator(self, mach):
        """returns pitching moment curve of elevator wrt deflection angle."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        downwash = LiftingSurface(wing).d_epsilon_d_alpha(ht, mach)  # []
        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []
        tau = self.plane['elevator']['chord_ratio']
        c_m_de = - c_l_alpha_ht * s_ht / s_w * (1 - downwash) * (x_ac_ht_bar - cg_bar) * tau  # [1/rad]
        return c_m_de

    def c_m_pitch_rate(self, mach):
        """returns pitching moment curve of elevator wrt pitch rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []
        c_m_q = - 2 * c_l_alpha_ht * s_ht / s_w * (x_ac_ht_bar - cg_bar) ** 2
        return c_m_q

    def c_m_alpha_dot(self, mach):
        """returns pitch moment curve of elevator wrt alpha rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = mac(wing['aspect_ratio'], s_w, wing['taper'])  # [ft]
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        x_ac_ht_bar = x_ac_ht / c_bar  # []
        downwash = LiftingSurface(wing).d_epsilon_d_alpha(ht, mach)  # []
        c_m_adt = - 2 * c_l_alpha_ht * s_ht / s_w * downwash * (x_ac_ht_bar - cg_bar) ** 2
        return c_m_adt

    def c_d_zero(self, mach, altitude):
        """returns faired drag coefficient."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        vt = self.plane['vertical']
        s_vt = vt['planform']  # [ft^2]
        c_d_0_w = LiftingSurface(wing).parasite_drag(mach, altitude)
        c_d_0_ht = LiftingSurface(ht).parasite_drag(mach, altitude) * s_ht / s_w
        c_d_0_vt = LiftingSurface(vt).parasite_drag(mach, altitude) * s_vt / s_w
        c_d_0_f = Fuselage(self.plane).parasite_drag_fuselage(mach, altitude)
        c_d_0 = c_d_0_w + c_d_0_ht + c_d_0_vt + c_d_0_f
        return c_d_0

    def c_y_beta(self, mach):
        """returns side force coefficient wrt sideslip."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        vt = self.plane['vertical']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        s_v = vt['planform']
        z_d = (self.plane['wing']['waterline']-self.plane['weight']['cg'][2])/self.plane['fuselage']['height']
        k_int = interp(z_d, [-1, 0, 1], [1.8, 0, 1.5])
        c_y_b_w = -0.0001*abs(wing['dihedral'])*180/pi
        c_y_b_b = -2 * k_int * s_v / s_w
        c_y_b_ht = -0.0001*abs(wing['dihedral'])*180/pi * (
                LiftingSurface(wing).d_sigma_d_beta_ht(ht, self.plane['fuselage'])*s_ht/s_w)
        c_y_b_vt = -k_yv*abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage'])*s_v/s_w)
        c_y_b = c_y_b_b + c_y_b_w + c_y_b_ht + c_y_b_vt
        return c_y_b

    def c_y_beta_dot(self):
        """returns side force coefficient wrt sideslip rate."""
        c_y_b_dt = 0 * self.plane['vertical']['planform']
        return c_y_b_dt

    def c_y_roll_rate(self, mach, alpha):
        """returns side force coefficient wrt roll rate."""
        wing = self.plane['vertical']['wing']
        s_w = wing['planform']  # [ft^2]
        vt = self.plane['vertical']['vertical']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        s_v = vt['planform']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['vertical']['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['vertical']['weight']['cg'][0]
        b = span(wing['aspect_ratio'], s_w)
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        c_y_p = 2*c_y_b_vt*(z_v*cos(alpha)-x_v*sin(alpha))/b
        return c_y_p

    def c_y_yaw_rate(self, mach, alpha):
        """returns side force coefficient wrt yaw rate."""
        wing = self.plane['vertical']['wing']
        s_w = wing['planform']  # [ft^2]
        vt = self.plane['vertical']['vertical']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        s_v = vt['planform']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['vertical']['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['vertical']['weight']['cg'][0]
        b = span(wing['aspect_ratio'], s_w)
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        c_y_r = -2 * c_y_b_vt * (x_v * cos(alpha) + z_v * sin(alpha)) / b
        return c_y_r

    def c_y_delta_aileron(self):
        """returns side force coefficient wrt aileron deflection."""
        c_y_da = 0 * self.plane['vertical']['planform']
        return c_y_da

    def c_y_delta_rudder(self, mach):
        """returns side force coefficient wrt rudder deflection."""
        wing = self.plane['vertical']['wing']
        s_w = wing['planform']  # [ft^2]
        vt = self.plane['vertical']['vertical']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        s_v = vt['planform']
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        c_y_dr = (-c_y_b_vt * self.plane['vertical']['rudder']['chord_ratio'] *
                  self.plane['vertical']['rudder']['span_ratio'])
        return c_y_dr

    def c_r_beta(self, mach, alpha):
        """returns rolling moment coefficient wrt sideslip."""
        wing = self.plane['vertical']['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['vertical']['horizontal']
        s_h = ht['planform']
        vt = self.plane['vertical']['vertical']
        s_v = vt['planform']
        b = span(wing['aspect_ratio'], s_w)
        b_h = span(ht['aspect_ratio'], s_h)
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        taper = wing['taper']
        taper_h = ht['taper']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['vertical']['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['vertical']['weight']['cg'][0]
        c_r_b_w_1 = -1/6*c_l_alpha_w*(1+2*taper)/(1+taper)*wing['dihedral']*pi/180
        c_r_b_w_2 = 0.00917 * (wing['waterline'] / (self.plane['vertical']['fuselage']['height'] / 2) - 1)
        c_r_b_w_3 = - c_l_alpha_w * alpha * (1+2*taper)/(1+taper)*tan(deg2rad(wing['sweep_LE']))
        c_r_b_w = c_r_b_w_1 + c_r_b_w_2 + c_r_b_w_3
        c_r_b_ht = -1/6*c_l_alpha_ht*(1+2*taper_h)/(1+taper_h)*ht['dihedral']*pi/180*s_h*b_h/(s_w*b)
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        c_r_b_vt = c_y_b_vt*(z_v*cos(alpha)-x_v*sin(alpha))/b
        c_r_b = c_r_b_w+c_r_b_ht+c_r_b_vt
        return c_r_b

    def c_r_beta_dot(self):
        """returns rolling moment coefficient wrt sideslip rate."""
        c_r_b_dt = 0 * self.plane['vertical']['planform']
        return c_r_b_dt

    def c_r_roll_rate(self, mach):
        """returns rolling moment coefficient wrt roll rate."""
        wing = self.plane['vertical']['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['vertical']['horizontal']
        s_h = ht['planform']
        vt = self.plane['vertical']['vertical']
        s_v = vt['planform']
        b = span(wing['aspect_ratio'], s_w)
        b_h = span(ht['aspect_ratio'], s_h)
        taper = wing['taper']
        taper_h = ht['taper']
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        c_l_alpha_ht = LiftingSurface(ht).c_l_alpha_wing(mach)  # [1/rad]
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        c_r_p_w = -c_l_alpha_w*(1+3*taper)/(12*(1+taper))
        c_r_p_h = -c_l_alpha_ht*(1+3*taper_h)/(12*(1+taper_h))*s_h/s_w*(b_h/b)**2
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        c_r_p_v = 2*c_y_b_vt*(z_v/b)**2
        c_r_p = c_r_p_w + c_r_p_h + c_r_p_v
        return c_r_p

    def c_r_yaw_rate(self, mach, alpha):
        """returns rolling moment coefficient wrt yaw rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = span(wing['aspect_ratio'], s_w)
        vt = self.plane['vertical']
        s_v = vt['planform']
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        taper = wing['taper']
        c_r_r_w = c_l_alpha_w*alpha*(1+3*taper)/(6*(1+taper))
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_r_r_v = -2*c_y_b_vt*(x_v*cos(alpha)+z_v*sin(alpha))*(z_v*cos(alpha)-x_v*sin(alpha))/(b**2)
        c_r_r = c_r_r_v + c_r_r_w
        return c_r_r

    def c_r_delta_aileron(self, mach):
        """returns rolling moment coefficient wrt aileron deflection."""
        wing = self.plane['wing']
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        taper = wing['taper']
        a_l = self.plane['aileron_l']
        a_r = self.plane['aileron_r']
        t_al = a_l['chord_ratio']
        t_ar = a_r['chord_ratio']
        x_2_l = a_l['x_2_b']
        x_2_r = a_r['x_2_b']
        x_1_l = a_l['x_1_b']
        x_1_r = a_r['x_1_b']
        c_l_da = c_l_alpha_w*(t_al*(3*(x_2_l**2 - x_1_l**2)-2*(1-taper)*(x_2_l**3 - x_1_l**3))/(12*(1+taper)) +
                              t_ar*(3*(x_2_r**2 - x_1_r**2)-2*(1-taper)*(x_2_r**3 - x_1_r**3))/(12*(1+taper)))
        return c_l_da

    def c_r_delta_rudder(self, mach, alpha):
        """returns rolling moment coefficient wrt rudder deflection."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        vt = self.plane['vertical']
        b = span(wing['aspect_ratio'], s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_y_dr = self.plane.c_y_delta_rudder(mach)
        c_l_dr = c_y_dr*(z_v*cos(alpha)-x_v*sin(alpha))/b
        return c_l_dr

    def c_n_beta(self, mach, alpha):
        """returns yawing moment coefficient wrt sideslip."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = span(wing['aspect_ratio'], s_w)
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_y_b_w = -0.0001 * abs(wing['dihedral']) * 180 / pi
        vt = self.plane['vertical']
        s_v = vt['planform']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        y_w = y_mac(wing['aspect_ratio'], wing['planform'], wing['taper'])
        x_w = x_mac(y_w, wing['sweep_LE'])
        y_h = y_mac(ht['aspect_ratio'], ht['planform'], ht['taper'])
        x_h = x_mac(y_h, ht['sweep_LE'])
        x_w = wing['station'] + x_w - self.plane['weight']['cg'][0]
        z_w = wing['waterline'] - self.plane['weight']['cg'][2]
        x_h = ht['station'] + x_h - self.plane['weight']['cg'][0]
        z_h = ht['waterline'] - self.plane['weight']['cg'][2]
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_y_b_ht = (-0.0001 * abs(wing['dihedral']) * 180 / pi *
                    LiftingSurface(wing).d_sigma_d_beta_ht(ht, self.plane['fuselage']) * s_ht / s_w)
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        c_n_b_w = -c_y_b_w * (x_w * cos(alpha) + z_w * sin(alpha)) / b
        c_n_b_h = -c_y_b_ht*(x_h*cos(alpha)+z_h*sin(alpha))/b
        c_n_b_v = -c_y_b_vt*(x_v*cos(alpha)+z_v*sin(alpha))/b
        c_n_b = c_n_b_w+c_n_b_h+c_n_b_v
        return c_n_b

    def c_n_beta_dot(self):
        """returns yawing moment coefficient wrt sideslip rate."""
        c_n_b_dt = 0 * self.plane['vertical']['planform']
        return c_n_b_dt

    def c_n_roll_rate(self, mach, alpha):
        """returns yawing moment coefficient wrt roll rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = span(wing['aspect_ratio'], s_w)
        vt = self.plane['vertical']
        s_v = vt['planform']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        taper = wing['taper']
        c_n_p_w = -c_l_alpha_w * alpha * (1 + 3 * taper) / (12 * (1 + taper))
        c_n_p_v = -2*c_y_b_vt*(x_v*cos(alpha)+z_v*sin(alpha))*(z_v*cos(alpha)-x_v*sin(alpha)-z_v)/(b**2)
        c_n_p = c_n_p_w + c_n_p_v
        return c_n_p

    def c_n_yaw_rate(self, mach, alpha):
        """returns yawing moment coefficient wrt yaw rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = span(wing['aspect_ratio'], s_w)
        vt = self.plane['vertical']
        s_v = vt['planform']
        c_l_alpha_vt = LiftingSurface(vt).c_l_alpha_wing(mach)  # [1/rad]
        c_y_b_vt = -k_yv * abs(c_l_alpha_vt) * (
                LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage']) * s_v / s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_n_r = 2*c_y_b_vt*((x_v*cos(alpha)+z_v*sin(alpha))/b)**2
        return c_n_r

    def c_n_delta_aileron(self, mach):
        """returns yawing moment coefficient wrt aileron deflection."""
        wing = self.plane['wing']
        c_l_alpha_w = LiftingSurface(wing).c_l_alpha_wing(mach)  # [1/rad]
        taper = wing['taper']
        a_l = self.plane['aileron_l']
        a_r = self.plane['aileron_r']
        t_al = a_l['chord_ratio']
        t_ar = a_r['chord_ratio']
        x_2_l = a_l['x_2_b']
        x_2_r = a_r['x_2_b']
        x_1_l = a_l['x_1_b']
        x_1_r = a_r['x_1_b']
        c_n_da = -2*c_l_alpha_w/(12*(1+taper))*((t_al**2)*(3*(x_2_l**2 - x_1_l**2)-2*(1-taper)*(x_2_l**3 - x_1_l**3)) +
                                                (t_ar**2)*(3*(x_2_r**2 - x_1_r**2)-2*(1-taper)*(x_2_r**3 - x_1_r**3)))
        return c_n_da

    def c_n_delta_rudder(self, mach, alpha):
        """returns yawing moment coefficient wrt rudder deflection."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = span(wing['aspect_ratio'], s_w)
        vt = self.plane['vertical']
        c_y_dr = self.c_y_delta_rudder(mach)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_n_dr = -c_y_dr*(x_v*cos(alpha)+z_v*sin(alpha))/b
        return c_n_dr
