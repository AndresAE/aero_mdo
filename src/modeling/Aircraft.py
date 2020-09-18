"""Returns force and moment coefficients for total aircraft."""
from numpy import array, deg2rad, interp, pi, sin, cos, tan
from src.common.rotations import ned_to_body
from src.modeling.flap import c_f_m_flap
from src.modeling.Fuselage import Fuselage
from src.modeling.LiftingSurface import LiftingSurface
from src.modeling.trapezoidal_wing import mac, span, x_mac, y_mac
k_yv = 1


class Aircraft:
    def __init__(self, aircraft, mach):
        self.plane = aircraft
        self.mach = mach
        self.c_l_alpha_w = LiftingSurface(self.plane['wing']).c_l_alpha_wing(mach)  # [1/rad]
        self.c_l_alpha_ht = LiftingSurface(self.plane['horizontal']).c_l_alpha_wing(mach)  # [1/rad]
        self.c_l_alpha_vt = LiftingSurface(self.plane['vertical']).c_l_alpha_wing(mach)  # [1/rad]

        self.s_w = self.plane['wing']['planform']  # [ft^2]
        self.b = span(self.plane['wing']['aspect_ratio'], self.s_w)
        self.c_bar = mac(self.plane['wing']['aspect_ratio'], self.s_w, self.plane['wing']['taper'])  # [ft]

        cg = self.plane['weight']['cg']
        wing = self.plane['wing']
        ht = self.plane['horizontal']
        vt = self.plane['vertical']
        s_ht = ht['planform']  # [ft^2]
        s_vt = vt['planform']  # [ft^2]
        s_w = wing['planform']  # [ft^2]

        self.downwash = LiftingSurface(wing).d_epsilon_d_alpha(ht, mach)
        self.d_sigma_d_beta_ht = LiftingSurface(wing).d_sigma_d_beta_ht(ht, self.plane['fuselage'])
        self.d_sigma_d_beta_vt = LiftingSurface(wing).d_sigma_d_beta_vt(vt, self.plane['fuselage'])
        self.x_ac_ht = LiftingSurface(ht).aerodynamic_center()  # [ft]
        self.x_ac_w = LiftingSurface(wing).aerodynamic_center()  # [ft]

        # elevator
        cfm_l = c_f_m_flap(ht, ht['control_1'], mach, cg)
        cfm_r = c_f_m_flap(ht, ht['control_2'], mach, cg)
        self.cfm_de = (cfm_l + cfm_r) * s_ht / s_w
        self.cfm_de[3:6] = self.cfm_de[3:6] / array([self.b, self.c_bar, self.b])

        # ailerons
        cfm_l = c_f_m_flap(wing, wing['control_1'], mach, cg)
        cfm_r = c_f_m_flap(wing, wing['control_4'], mach, cg)
        self.cfm_da = (cfm_l + cfm_r * -1)
        self.cfm_da[3:6] = self.cfm_da[3:6] / array([self.b, self.c_bar, self.b])

        # rudder
        cfm_r = c_f_m_flap(vt, vt['control_1'], mach, cg)
        self.cfm_dr = cfm_r * s_vt / s_w * -1
        r = ned_to_body(pi / 2, 0, 0)
        self.cfm_dr[0:3] = r @ self.cfm_dr[0:3]
        self.cfm_dr[3:6] = r @ self.cfm_dr[3:6]
        self.cfm_dr[3:6] = self.cfm_dr[3:6] / array([self.b, self.c_bar, self.b])

    def c_l_zero(self):
        """baseline lift coefficient."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_0_w = self.c_l_alpha_w * (deg2rad(wing['incidence'] - wing['alpha_zero_lift']))  # []
        epsilon = 2 * c_l_0_w / (pi * wing['aspect_ratio'])  # [rad]
        c_l_0_ht = self.c_l_alpha_ht * s_ht / s_w * (deg2rad(ht['incidence'] - ht['alpha_zero_lift'] - epsilon))  # []
        c_l_0 = c_l_0_w + c_l_0_ht  # []
        return c_l_0

    def c_l_alpha(self):
        """returns lift curve slope for aircraft."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_l_alpha_airplane = self.c_l_alpha_w + self.c_l_alpha_ht*s_ht/s_w*(1-self.downwash)  # [1/rad]
        return c_l_alpha_airplane

    def c_l_delta_elevator(self):
        """returns lift curve of elevator wrt deflection angle."""
        return -self.cfm_de[2]

    def c_l_pitch_rate(self):
        """returns lift curve of elevator wrt pitch rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = self.c_bar
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        x_ac_ht_bar = self.x_ac_ht / c_bar  # []
        c_l_q = 2 * self.c_l_alpha_ht * s_ht / s_w * (x_ac_ht_bar - cg_bar)
        return c_l_q

    def c_l_alpha_dot(self):
        """returns lift curve of elevator wrt alpha rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = self.c_bar
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        x_ac_ht_bar = self.x_ac_ht / c_bar  # []
        c_l_adt = 2 * self.c_l_alpha_ht * s_ht / s_w * (x_ac_ht_bar - cg_bar) * self.downwash
        return c_l_adt

    def c_m_zero(self, altitude):
        """baseline lift coefficient."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        vt = self.plane['vertical']
        s_vt = vt['planform']  # [ft^2]

        c_bar = self.c_bar
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        z_cg = self.plane['weight']['cg'][2]
        x_ac_w_bar = self.x_ac_w / c_bar  # []
        x_ac_ht_bar = self.x_ac_ht / c_bar  # []

        c_l_0_w = self.c_l_alpha_w * (deg2rad(wing['incidence'] - wing['alpha_zero_lift']))  # []
        c_m_0_w = c_l_0_w * (cg_bar - x_ac_w_bar)
        epsilon = 2 * c_l_0_w / (pi * wing['aspect_ratio'])  # [rad]
        c_l_0_ht = self.c_l_alpha_ht * s_ht / s_w * (deg2rad(ht['incidence'] - ht['alpha_zero_lift'] - epsilon))  # []

        z_w = (z_cg - wing['waterline']) / c_bar
        z_ht = (z_cg - ht['waterline']) / c_bar
        z_vt = (z_cg - vt['waterline']) / c_bar
        z_f = (z_cg - self.plane['fuselage']['height'] / 2) / c_bar

        c_m_0_w_d = - LiftingSurface(wing).parasite_drag(self.mach, altitude) * z_w
        c_m_0_ht_d = - LiftingSurface(ht).parasite_drag(self.mach, altitude) * s_ht / s_w * z_ht
        c_m_0_vt_d = - LiftingSurface(vt).parasite_drag(self.mach, altitude) * s_vt / s_w * z_vt
        c_m_0_f_d = - Fuselage(self.plane).parasite_drag_fuselage(self.mach, altitude) * z_f
        c_m_0_ht = c_l_0_ht * (x_ac_ht_bar - cg_bar)
        c_m_0 = (wing['airfoil_cm0'] + ht['airfoil_cm0'] + c_m_0_w + c_m_0_ht
                 + c_m_0_w_d + c_m_0_ht_d + c_m_0_vt_d + c_m_0_f_d)  # []
        return c_m_0

    def c_m_alpha(self):
        """returns pitching moment curve slope for aircraft."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = self.c_bar
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        x_ac_w_bar = self.x_ac_w / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        x_ac_ht_bar = self.x_ac_ht / c_bar  # []
        c_m_alpha_w = self.c_l_alpha_w * (cg_bar - x_ac_w_bar)  # [1/rad]
        c_m_alpha_ht = self.c_l_alpha_ht * s_ht / s_w * (1 - self.downwash) * (x_ac_ht_bar - cg_bar)  # [1/rad]
        c_m_alpha_airplane = c_m_alpha_w - c_m_alpha_ht  # [1/rad]
        return c_m_alpha_airplane

    def c_m_delta_elevator(self):
        """returns pitching moment curve of elevator wrt deflection angle."""
        return self.cfm_de[4]

    def c_m_pitch_rate(self):
        """returns pitching moment curve of elevator wrt pitch rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = self.c_bar
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        x_ac_ht_bar = self.x_ac_ht / c_bar  # []
        c_m_q = - 2 * self.c_l_alpha_ht * s_ht / s_w * (x_ac_ht_bar - cg_bar) ** 2
        return c_m_q

    def c_m_alpha_dot(self):
        """returns pitch moment curve of elevator wrt alpha rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        c_bar = self.c_bar
        cg_bar = self.plane['weight']['cg'][0] / c_bar  # []
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        x_ac_ht_bar = self.x_ac_ht / c_bar  # []
        c_m_adt = - 2 * self.c_l_alpha_ht * s_ht / s_w * self.downwash * (x_ac_ht_bar - cg_bar) ** 2
        return c_m_adt

    def c_d_zero(self, altitude):
        """returns faired drag coefficient."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        vt = self.plane['vertical']
        s_vt = vt['planform']  # [ft^2]
        c_d_0_w = LiftingSurface(wing).parasite_drag(self.mach, altitude)
        c_d_0_ht = LiftingSurface(ht).parasite_drag(self.mach, altitude) * s_ht / s_w
        c_d_0_vt = LiftingSurface(vt).parasite_drag(self.mach, altitude) * s_vt / s_w
        c_d_0_f = Fuselage(self.plane).parasite_drag_fuselage(self.mach, altitude)
        c_d_0 = c_d_0_w + c_d_0_ht + c_d_0_vt + c_d_0_f
        return c_d_0

    def c_y_beta(self):
        """returns side force coefficient wrt sideslip."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        vt = self.plane['vertical']
        s_v = vt['planform']
        z_d = (self.plane['wing']['waterline']-self.plane['weight']['cg'][2])/self.plane['fuselage']['height']
        k_int = interp(z_d, [-1, 0, 1], [1.8, 0, 1.5])
        c_y_b_w = -0.0001*abs(wing['dihedral'])*180/pi
        c_y_b_b = -2 * k_int * s_v / s_w
        c_y_b_ht = -0.0001*abs(wing['dihedral'])*180/pi * (self.d_sigma_d_beta_ht*s_ht/s_w)
        c_y_b_vt = -k_yv*abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt*s_v/s_w)
        c_y_b = c_y_b_b + c_y_b_w + c_y_b_ht + c_y_b_vt
        return c_y_b

    def c_y_beta_dot(self):
        """returns side force coefficient wrt sideslip rate."""
        c_y_b_dt = 0 * self.plane['vertical']['planform']
        return c_y_b_dt

    def c_y_roll_rate(self, alpha):
        """returns side force coefficient wrt roll rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        vt = self.plane['vertical']
        s_v = vt['planform']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        b = span(wing['aspect_ratio'], s_w)
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        c_y_p = 2*c_y_b_vt*(z_v*cos(alpha)-x_v*sin(alpha))/b
        return c_y_p

    def c_y_yaw_rate(self, alpha):
        """returns side force coefficient wrt yaw rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        vt = self.plane['vertical']
        s_v = vt['planform']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        b = span(wing['aspect_ratio'], s_w)
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        c_y_r = -2 * c_y_b_vt * (x_v * cos(alpha) + z_v * sin(alpha)) / b
        return c_y_r

    def c_y_delta_aileron(self):
        """returns side force coefficient wrt aileron deflection."""
        return self.cfm_da[1]

    def c_y_delta_rudder(self):
        """returns side force coefficient wrt rudder deflection."""
        return self.cfm_dr[1]

    def c_r_beta(self, alpha):
        """returns rolling moment coefficient wrt sideslip."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_h = ht['planform']
        vt = self.plane['vertical']
        s_v = vt['planform']
        b = self.b
        b_h = span(ht['aspect_ratio'], s_h)
        taper = wing['taper']
        taper_h = ht['taper']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_r_b_w_1 = -1/6*self.c_l_alpha_w*(1+2*taper)/(1+taper)*wing['dihedral']*pi/180
        c_r_b_w_2 = 0.00917 * (wing['waterline'] / (self.plane['fuselage']['height'] / 2) - 1)
        c_r_b_w_3 = - self.c_l_alpha_w * alpha * (1+2*taper)/(1+taper)*tan(deg2rad(wing['sweep_LE']))
        c_r_b_w = c_r_b_w_1 + c_r_b_w_2 + c_r_b_w_3
        c_r_b_ht = -1/6*self.c_l_alpha_ht*(1+2*taper_h)/(1+taper_h)*ht['dihedral']*pi/180*s_h*b_h/(s_w*b)
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        c_r_b_vt = c_y_b_vt*(z_v*cos(alpha)-x_v*sin(alpha))/b
        c_r_b = c_r_b_w+c_r_b_ht+c_r_b_vt
        return c_r_b

    def c_r_beta_dot(self):
        """returns rolling moment coefficient wrt sideslip rate."""
        c_r_b_dt = 0 * self.plane['vertical']['planform']
        return c_r_b_dt

    def c_r_roll_rate(self):
        """returns rolling moment coefficient wrt roll rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        ht = self.plane['horizontal']
        s_h = ht['planform']
        vt = self.plane['vertical']
        s_v = vt['planform']
        b = self.b
        b_h = span(ht['aspect_ratio'], s_h)
        taper = wing['taper']
        taper_h = ht['taper']
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        c_r_p_w = -self.c_l_alpha_w*(1+3*taper)/(12*(1+taper))
        c_r_p_h = -self.c_l_alpha_ht*(1+3*taper_h)/(12*(1+taper_h))*s_h/s_w*(b_h/b)**2
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        c_r_p_v = 2*c_y_b_vt*(z_v/b)**2
        c_r_p = c_r_p_w + c_r_p_h + c_r_p_v
        return c_r_p

    def c_r_yaw_rate(self, alpha):
        """returns rolling moment coefficient wrt yaw rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = self.b
        vt = self.plane['vertical']
        s_v = vt['planform']
        taper = wing['taper']
        c_r_r_w = self.c_l_alpha_w*alpha*(1+3*taper)/(6*(1+taper))
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_r_r_v = -2*c_y_b_vt*(x_v*cos(alpha)+z_v*sin(alpha))*(z_v*cos(alpha)-x_v*sin(alpha))/(b**2)
        c_r_r = c_r_r_v + c_r_r_w
        return c_r_r

    def c_r_delta_aileron(self):
        """returns rolling moment coefficient wrt aileron deflection."""
        return self.cfm_da[3]

    def c_r_delta_rudder(self):
        """returns rolling moment coefficient wrt rudder deflection."""
        return self.cfm_dr[3]

    def c_n_beta(self, alpha):
        """returns yawing moment coefficient wrt sideslip."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = self.b
        ht = self.plane['horizontal']
        s_ht = ht['planform']  # [ft^2]
        c_y_b_w = -0.0001 * abs(wing['dihedral']) * 180 / pi
        vt = self.plane['vertical']
        s_v = vt['planform']
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
        c_y_b_ht = (-0.0001 * abs(wing['dihedral']) * 180 / pi * self.d_sigma_d_beta_ht * s_ht / s_w)
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        c_n_b_w = -c_y_b_w * (x_w * cos(alpha) + z_w * sin(alpha)) / b
        c_n_b_h = -c_y_b_ht*(x_h*cos(alpha)+z_h*sin(alpha))/b
        c_n_b_v = -c_y_b_vt*(x_v*cos(alpha)+z_v*sin(alpha))/b
        c_n_b = c_n_b_w+c_n_b_h+c_n_b_v
        return c_n_b

    def c_n_beta_dot(self):
        """returns yawing moment coefficient wrt sideslip rate."""
        c_n_b_dt = 0 * self.plane['vertical']['planform']
        return c_n_b_dt

    def c_n_roll_rate(self, alpha):
        """returns yawing moment coefficient wrt roll rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = self.b
        vt = self.plane['vertical']
        s_v = vt['planform']
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        taper = wing['taper']
        c_n_p_w = -self.c_l_alpha_w * alpha * (1 + 3 * taper) / (12 * (1 + taper))
        c_n_p_v = -2*c_y_b_vt*(x_v*cos(alpha)+z_v*sin(alpha))*(z_v*cos(alpha)-x_v*sin(alpha)-z_v)/(b**2)
        c_n_p = c_n_p_w + c_n_p_v
        return c_n_p

    def c_n_yaw_rate(self, alpha):
        """returns yawing moment coefficient wrt yaw rate."""
        wing = self.plane['wing']
        s_w = wing['planform']  # [ft^2]
        b = self.b
        vt = self.plane['vertical']
        s_v = vt['planform']
        c_y_b_vt = -k_yv * abs(self.c_l_alpha_vt) * (self.d_sigma_d_beta_vt * s_v / s_w)
        z_vt = y_mac(vt['aspect_ratio'], vt['planform'], vt['taper'])
        x_vt = x_mac(z_vt, vt['sweep_LE'])
        z_v = vt['waterline'] + z_vt - self.plane['weight']['cg'][2]
        x_v = vt['station'] + x_vt - self.plane['weight']['cg'][0]
        c_n_r = 2*c_y_b_vt*((x_v*cos(alpha)+z_v*sin(alpha))/b)**2
        return c_n_r

    def c_n_delta_aileron(self):
        """returns yawing moment coefficient wrt aileron deflection."""
        return self.cfm_da[5]

    def c_n_delta_rudder(self):
        """returns yawing moment coefficient wrt rudder deflection."""
        return self.cfm_dr[5]
