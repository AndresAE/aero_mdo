from numpy import array, cos as c, sin as s, tan as t


def ned_to_body(phi, theta, psi):
    """North East Down to Body axis direct cosine matrix."""
    b = array([[c(theta)*c(psi), c(theta)*s(psi), -s(theta)],
               [-c(phi)*s(psi)+s(phi)*s(theta)*c(psi), c(phi)*c(psi)+s(phi)*s(theta)*s(psi), s(phi)*c(theta)],
               [s(phi)*s(psi)+c(phi)*s(theta)*c(psi), -s(phi)*c(psi)+c(phi)*s(theta)*s(psi), c(phi)*c(theta)]
               ])
    return b


def eci_to_ned(mu, l):
    """Earth Centered Inertial to North East Down direct cosine matrix."""
    b = array([[c(mu), -s(mu)*s(l), s(mu)*c(l)],
               [0, c(l), s(l)],
               [-s(mu), -c(mu)*s(l), c(mu)*c(l)]
               ])
    return b


def body_to_wind(a, b):
    """Earth Centered Inertial to North East Down direct cosine matrix."""
    r = array([[c(a)*c(b), s(b), s(a)*c(b)],
               [-c(a)*s(b), c(b), -s(a)*s(b)],
               [-s(a), 0, c(a)]
               ])
    return r


def angular_rate_rotation(phi, theta):
    """Body Axis Rate to Euler Rate rotation matrix."""
    b = array([[1, t(theta)*s(phi), t(theta)*c(phi)],
               [0, c(phi), -s(phi)],
               [0, s(phi)/c(theta), c(phi)/c(theta)]
               ])
    return b
