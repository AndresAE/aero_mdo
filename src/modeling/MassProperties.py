from numpy import array, ceil, cos, deg2rad, pi
from src.common import Atmosphere, Earth, constants
from src.modeling import LiftingSurface
from src.modeling.Propulsion import propeller
from src.modeling.trapezoidal_wing import span, sweep_x
g = Earth(0).gravity()  # [f/s2]


class MassProperties:
    def __init__(self, aircraft):
        self.aircraft = aircraft

    def weight_simple(self):
        """return empirical aircraft weight estimate."""
        s_w = self.aircraft['wing']['planform']
        pax = self.aircraft['fuselage']['pax']
        w_s = 117.8 * s_w - 24298
        w_pax = (480.1 + 250) * pax + 1049.1 + self.aircraft['propulsion']['battery_weight']
        w = (w_s + w_pax) / 2
        return w

    def i_xx_simple(self):
        """return empirical aircraft x axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        i = (w / g) * (0.3 * b / 2) ** 2
        return i

    def i_yy_simple(self):
        """return empirical aircraft y axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        d = self.aircraft['fuselage']['length']
        i = (w / g) * (0.3 * d / 2) ** 2
        return i

    def i_zz_simple(self):
        """return empirical aircraft z axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        d = self.aircraft['fuselage']['length']
        e = (b + d) / 2
        i = (w / g) * (0.4 * e / 2) ** 2
        return i

    def i_xz_simple(self):
        """return empirical aircraft xz axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        i_xz = ((w / g) * (0.3 * b / 2) ** 2) / 3
        return i_xz

    def weight_buildup(self, requirements, tol=10e-2):
        """return component buildup mass properties."""
        mtow = self.aircraft['weight']['weight']
        aircraft = self.aircraft
        r = requirements['performance']['range']
        mach = requirements['performance']['cruise_mach']
        n = max(abs(array(requirements['loads']['n_z'])))

        res = 10
        w_ai = anti_icing_weight(aircraft)
        w_avi = avionics_weight(aircraft, r)
        w_car = cargo_weight(aircraft)
        w_ele = electrical_weight(aircraft)
        w_fus = fuselage_weight(aircraft)
        w_pnt = paint(aircraft)
        w_pax = passenger_weight(aircraft)
        w_prp = propulsion_weight(aircraft)
        w_ful = aircraft['propulsion']['fuel_mass'] * g
        w_fixed = w_avi + w_car + w_ele + w_fus + w_pnt + w_pax + w_prp + w_ful + w_ai
        w_fcs = 0
        w_ht = 0
        w_mg = 0
        w_ng = 0
        w_vt = 0
        w_w = 0
        while abs(res) > tol:
            w_fcs = fcs_weight(aircraft, mtow, mach)
            w_ht = ht_weight(aircraft, mtow)
            w_mg = main_gear_weight(aircraft, mtow)
            w_ng = nose_gear_weight(aircraft, mtow)
            w_vt = vt_weight(aircraft, mtow)
            w_w = wing_weight(aircraft, mtow, mach, n)
            w_iter = w_fcs + w_ht + w_mg + w_ng + w_vt + w_w
            mtow_out = w_iter + w_fixed
            res = mtow - mtow_out
            mtow = mtow_out
        p_prp = array([aircraft['wing']['station'], 0, aircraft['wing']['waterline']])
        p_w = array([LiftingSurface(aircraft['wing']).aerodynamic_center(c=0.5), aircraft['wing']['buttline'],
                     aircraft['wing']['waterline']])
        p_ht = array([LiftingSurface(aircraft['horizontal']).aerodynamic_center(c=0.5),
                      aircraft['horizontal']['buttline'], aircraft['horizontal']['waterline']])
        p_vt = array([LiftingSurface(aircraft['vertical']).aerodynamic_center(c=0.5),
                      aircraft['vertical']['buttline'], aircraft['vertical']['waterline']])
        p_mg = array([aircraft['landing_gear']['main'][0], 0, aircraft['landing_gear']['main'][2] / 2])
        p_ng = array([aircraft['landing_gear']['nose'][0], aircraft['landing_gear']['nose'][1],
                      aircraft['landing_gear']['nose'][2] / 2])
        p_fus = array([aircraft['fuselage']['length'] / 2, 0, aircraft['fuselage']['width']])
        w_r = (p_w * (w_fcs + w_w + w_ful) + p_prp * w_prp +
               p_ht * w_ht + p_vt * w_vt + p_mg * w_mg + p_ng * w_ng +
               p_fus * (w_fus + w_avi + w_ele + w_pnt + w_pax))
        cg = w_r / mtow

        # j = inertia tensor
        # cg = [x, y, z]
        return mtow, cg


def anti_icing_weight(aircraft):
    """return cargo/luggage weight."""
    w = aircraft['wing']
    b = span(w['aspect_ratio'], w['planform'])
    sweep = w['sweep_LE']
    w_ai = b / cos(deg2rad(sweep)) + 3.8 * aircraft['propulsion']['n_engines'] + 1.5 * aircraft['fuselage']['width']
    return w_ai


def avionics_weight(aircraft, r):
    """return wing weight."""
    n_pax = aircraft['fuselage']['pax']
    n_fc = n_crew(aircraft)
    fus = aircraft['fuselage']
    h = fus['height']
    s_fus = fus['length'] * fus['width']
    w_avi = 15.8 * (r ** 0.1) * (n_fc ** 0.7) * (s_fus ** 0.43)
    w_ac = (3.2 * ((s_fus * h) ** 0.6) + 9 * n_pax ** 0.83)
    return w_avi + w_ac


def cargo_weight(aircraft):
    """return cargo/luggage weight."""
    n_pax = aircraft['fuselage']['pax']
    n_fa = n_crew(aircraft)
    w_cargo = constants.far_25_cargo_weight() * (n_pax + n_fa)
    return w_cargo


def electrical_weight(aircraft):
    """return electrical weight."""
    n_pax = aircraft['fuselage']['pax']
    n_fc = n_crew(aircraft)
    fus = aircraft['fuselage']
    w_ele = 92 * (fus['length'] ** 0.4) * (fus['width'] ** 0.14) * (1 ** 0.27) * (1 + 0.044 * n_fc + 0.0014 * n_pax)
    return w_ele


def fcs_weight(aircraft, mtow, mach):
    """return fcs weight."""
    w = aircraft['wing']
    ht = aircraft['horizontal']
    vt = aircraft['vertical']
    s_w = w['planform']
    s_ht = ht['planform']
    s_vt = vt['planform']
    s_a = (w['control_1']['b_2'] - w['control_1']['b_1']) * w['control_1']['cf_c'] * s_w
    s_e = (ht['control_2']['b_2'] - ht['control_2']['b_1']) * ht['control_2']['cf_c'] * s_ht
    s_r = (vt['control_1']['b_2'] - vt['control_1']['b_1']) * vt['control_1']['cf_c'] * s_vt
    s_flap = s_a + s_e + s_r
    if 'control_3' in w:
        s_f = (w['control_3']['b_2'] - w['control_3']['b_1']) * w['control_3']['cf_c'] * s_w
        s_flap = s_flap + s_f
    w_fcs = 1.1 * (mach ** 0.52) * (s_flap ** 0.6) * mtow ** 0.32
    return w_fcs


def fuselage_weight(aircraft):
    """return fuselage weight."""
    fus = aircraft['fuselage']
    w_fus = 1.5 * ((fus['length'] * fus['width']) ** 1.35)
    return w_fus


def ht_weight(aircraft, mtow):
    """return ht weight."""
    ht = aircraft['horizontal']
    w_ht = 0.53 * ht['planform'] * (ht['taper'] + 0.5) * mtow ** 0.2
    return w_ht


def main_gear_weight(aircraft, mtow):
    """return main landing gear weight."""
    mg = aircraft['landing_gear']['main']
    w_lg = 0.0117 * (mtow ** 0.95) * ((-mg[2]) ** 0.43)
    return w_lg


def n_crew(aircraft):
    """return passenger transport crew number."""
    n_pax = aircraft['fuselage']['pax']
    n = 2 + ceil(n_pax / 20)
    return n


def nose_gear_weight(aircraft, mtow):
    """return nose landing gear weight."""
    ng = aircraft['landing_gear']['nose']
    w_lg = 0.048 * (mtow ** 0.67) * ((-ng[2]) ** 0.43)
    return w_lg


def paint(aircraft):
    """return paint weight."""
    s_wet = 4 * aircraft['wing']['planform']
    w_p = constants.paint_density() * s_wet
    return w_p


def passenger_weight(aircraft):
    """return passenger weight, includes furnishings."""
    n_fc = n_crew(aircraft)
    n_pax = aircraft['fuselage']['pax']
    w_pax = (constants.far_25_passenger_weight() + constants.far_25_furnishing_weight()) * (n_pax + n_fc)
    return w_pax


def propulsion_weight(aircraft):
    """return engine weight."""
    w_p = []
    fus = aircraft['fuselage']
    w = aircraft['wing']
    b = span(w['aspect_ratio'], w['planform'])
    length = fus['length']
    for ii in range(0, aircraft['propulsion']['n_engines']):
        engine = aircraft['propulsion']["engine_%d" % (ii + 1)]
        if engine['type'] == 'prop':
            t = propeller(engine, [0, 0, 0], 0, 1)
            t = (sum(t[0:3] ** 2)) ** 0.5
            rho = Atmosphere(0).air_density()
            d = engine['diameter']
            a = pi * (d / 2) ** 2
            u_e = (2 * t / (rho * a)) ** 0.5
            u_disk = u_e / 2
            p = t * u_disk
            w_controls = 60.27 * ((length + b) * 10 ** -2) ** 0.724
            w_prop = 32 * (4 ** 0.391) * (d * p * constants.lbft_s2hp() * 10 ** -3) ** 0.782
            w_prop_control = 4.5 * (4 ** 0.379) * (d * p * constants.lbft_s2hp() * 10 ** -3) ** 0.759
            w_engine = p * constants.lbft_s2hp() / constants.electric_hp_lb()
            w_p.append(w_engine + w_prop + w_prop_control + w_controls)
        if engine['type'] == 'jet':
            w_controls = 88.46 * ((length + b) * 10 ** -2) ** 0.294
            w_engine = engine['thrust'] / constants.jet_lbt_lb()
            d_nac = 0.04 * engine['thrust'] ** 0.5
            l_nac = 0.07 * engine['thrust'] ** 0.5
            w_nac = 0.25 * d_nac * l_nac * engine['thrust'] ** 0.36
            w_p.append(w_engine + w_controls + w_nac)
    w_fuel_tank = 1.07 * (aircraft['propulsion']['fuel_mass'] * g) ** 0.58
    w_total = sum(w_p) + w_fuel_tank
    return w_total


def vt_weight(aircraft, mtow):
    """return vt weight."""
    vt = aircraft['vertical']
    w_ht = 0.32 * (vt['planform'] ** 0.85) * (vt['taper'] + 0.5) * mtow ** 0.3
    return w_ht


def wing_weight(aircraft, mtow, mach, n):
    """return wing weight."""
    w = aircraft['wing']
    tc = w['airfoil']
    tc = int(tc[-2:]) / 100
    sweep_2 = sweep_x(w['aspect_ratio'], w['taper'], w['sweep_LE'], 0.5)
    w_w = 0.006 * ((w['planform'] ** 0.48) * w['aspect_ratio'] * (mach ** 0.43) * ((mtow * n) ** 0.84) *
                     (w['taper'] ** 0.14) / (((100 * tc) ** 0.76) * cos(deg2rad(sweep_2)) ** 1.54))
    return w_w
