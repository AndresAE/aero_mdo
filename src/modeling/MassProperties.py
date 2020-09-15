from src.common import Earth
from src.modeling.trapezoidal_wing import span
g = Earth(0).gravity()  # [f/s2]


class LiftingSurface:
    def __init__(self, aircraft):
        self.aircraft = aircraft

    def weight(self):
        s_w = self.aircraft['wing']['planform']
        pax = self.aircraft['fuselage']['pax']
        w_s = 117.8 * s_w - 24298
        w_pax = (480.1 + 250) * pax + 1049.1 + self.aircraft['propulsion']['battery_weight']
        w = (w_s + w_pax) / 2
        return w

    def i_xx(self):
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        i = (w / g) * (0.3 * b / 2) ** 2
        return i

    def i_yy(self):
        w = self.aircraft['weight']['weight']
        d = self.aircraft['fuselage']['length']
        i = (w / g) * (0.3 * d / 2) ** 2
        return i

    def i_zz(self):
        w = self.aircraft['weight']['weight']
        b = span(self.aircraft['wing']['aspect_ratio'], self.aircraft['wing']['planform'])
        d = self.aircraft['fuselage']['length']
        e = (b + d) / 2
        i = (w / g) * (0.4 * e / 2) ** 2
        return i

    def i_xz(self):
        i_xz = 0 * self.aircraft['weight']['weight']
        return i_xz
