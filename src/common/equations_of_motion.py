# File containing Equations of Motion
from numpy import concatenate, cross, dot, identity, linalg, transpose, zeros
from src.common.rotations import angular_rate_rotation, ned_to_body
from src.modeling.force_model import c_f_m
dx = 0.01
du = 0.01


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


def nonlinear_eom_to_ss(aircraft, x_ss, u_ss, x_0, u_0, m, j):
    """aircraft system linearization routine"""
    x = x_0
    u = u_0
    a = zeros((len(x_0), len(x_0)))
    b = zeros((len(x_0), len(u_0)))
    for ii in range(0, len(x_0)):
        x[ii] = x[ii] + dx
        c = c_f_m(aircraft, x, u_0)
        dxdt_1 = nonlinear_eom(x, m, j, c)

        x[ii] = x[ii] - dx
        c = c_f_m(aircraft, x, u_0)
        dxdt_2 = nonlinear_eom(x, m, j, c)
        ddx_dx = (dxdt_1 - dxdt_2)/(2*dx)
        a[:, ii] = transpose(ddx_dx)
        x = x_0

    for ii in range(0, len(u_0)):
        u[ii] = u[ii] + du
        c = c_f_m(aircraft, x_0, u)
        dxdt_1 = nonlinear_eom(x, m, j, c)

        u[ii] = u[ii] - du
        c = c_f_m(aircraft, x_0, u)
        dxdt_2 = nonlinear_eom(x, m, j, c)
        ddx_dx = (dxdt_1 - dxdt_2)/(2*du)
        b[:, ii] = transpose(ddx_dx)
        u = u_0

    a_out = a[x_ss, :]
    a_out = a_out[:, x_ss]

    b_out = b[x_ss, :]
    b_out = b_out[:, u_ss]

    c_out = identity(len(x_ss))
    d_out = zeros((len(x_ss), len(u_ss)))
    return a_out, b_out, c_out, d_out
