from src.common import constants
r_0 = 6378137  # [m]
gravitational_const = 6.67408 * 10 ** (-11)
m_earth = 5.9728 * 10 ** 24  # [kg]


class Earth:
    def __init__(self, altitude):
        self.altitude = altitude

    def gravity(self):
        """return acceleration due to gravity density wrt altitude."""
        r = r_0 + self.altitude / constants.m2ft()
        g = constants.m2ft() * gravitational_const * m_earth / r ** 2
        return g

# Public Methods #######################################################################################################
