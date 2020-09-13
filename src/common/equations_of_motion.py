# File containing Equations of Motion
from numpy import concatenate, cross, dot, linalg, transpose
from src.common.Rotations import angular_rate_rotation, ned_to_body


def local_acceleration(p, cg, x, dxdt):
    """calculate acceleration of point on rigid body."""
    r_pcg = p*[-1, 1, -1] + cg
    omega = x[6:9]
    domega_dt = dxdt[6:9]
    dv_dt = dxdt[0:3]
    a_p = dv_dt + cross(domega_dt, r_pcg) + cross(omega, cross(omega, r_pcg))
    return a_p


def nonlinear_eom(x, m, j, c):
    """contains nonlinear equations of motion."""
    # x = [u v w phi theta psi p q r p_n p_e h]
    # m = mass of the system
    # j = inertia of the system
    # c = [f_x f_y f_z m_x m_y m_z]
    external_forces = c[0:3]
    external_moments = c[3:6]
    v = x[0:3]
    omega = x[6:9]
    euler = x[3:6]

    # linear momentum equations
    linear_momentum = external_forces / m - cross(omega, v)

    # kinematics equations
    b_euler = angular_rate_rotation(euler[0], euler[1])
    kinematics = b_euler @ omega

    # angular momentum equations
    angular_momentum = linalg.inv(j) @ transpose(external_moments - cross(omega, dot(j, omega)))

    # navigation equations
    b_body = ned_to_body(euler[0], euler[1], euler[2])
    navigation = b_body.transpose() @ v
    dx_dt = concatenate((concatenate((concatenate((linear_momentum, kinematics)), angular_momentum)), navigation))
    return dx_dt
