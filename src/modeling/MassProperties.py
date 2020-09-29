from numpy import pi, round
from src.common import Atmosphere, Earth, unit_conversions
from src.modeling.Propulsion import propeller
from src.modeling.trapezoidal_wing import span
g = Earth(0).gravity()  # [f/s2]


class MassProperties:
    def __init__(self, aircraft):
        self.aircraft = aircraft

    def weight(self):
        """return empirical aircraft weight estimate."""
        s_w = self.aircraft['wing']['planform']
        pax = self.aircraft['fuselage']['pax']
        w_s = 117.8 * s_w - 24298
        w_pax = (480.1 + 250) * pax + 1049.1 + self.aircraft['propulsion']['battery_weight']
        w = (w_s + w_pax) / 2
        return w

    def i_xx(self):
        """return empirical aircraft x axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        i = (w / g) * (0.3 * b / 2) ** 2
        return i

    def i_yy(self):
        """return empirical aircraft y axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        d = self.aircraft['fuselage']['length']
        i = (w / g) * (0.3 * d / 2) ** 2
        return i

    def i_zz(self):
        """return empirical aircraft z axis inertia estimate."""
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        d = self.aircraft['fuselage']['length']
        e = (b + d) / 2
        i = (w / g) * (0.4 * e / 2) ** 2
        return i

    def i_xz(self):
        """return empirical aircraft xz axis inertia estimate."""
        i_xz = 0 * self.aircraft['weight']['weight']
        return i_xz


def wing_weight():
    """return wing weight."""
    return 0


def fuselage_weight():
    """return wing weight."""
    return 0


def landing_gear_weight():
    """return wing weight."""
    return 0


def propulsion_weight(engine):
    """return engine weight."""
    t = propeller(engine, [0, 0, 0], 0, 1)
    t = (sum(t[0:3] ** 2)) ** 0.5
    rho = Atmosphere(0).air_density()
    a = pi * (engine['diameter'] / 2) ** 2
    u_e = (2 * t / (rho * a)) ** 0.5
    u_disk = u_e / 2
    p = t * u_disk
    w_p = p * unit_conversions.lbft_s2hp() / unit_conversions.electric_hp_lb()
    return w_p


def fcs_weight():
    """return wing weight."""
    return 0


def avionics_weight():
    """return wing weight."""
    return 0


def passenger_weight(n_pax):
    """return passenger weight, includes furnishings."""
    n_fa = round(n_pax / 20)
    w_pax = (200 + 33) * (n_pax + n_fa)
    return w_pax


def cargo_weight(n_pax):
    """return cargo/luggage weight."""
    n_fa = 2 + round(n_pax / 20)
    w_cargo = 50 * (n_pax + n_fa)
    return w_cargo
