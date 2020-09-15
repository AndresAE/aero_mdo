"""Returns force and moment coefficients for fuselages."""
from src.modeling.aerodynamics import friction_coefficient, pressure_drag


class Fuselage:
    def __init__(self, aircraft):
        self.plane = aircraft

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
