from common.equations_of_motion import nonlinear_eom
from numpy import identity, transpose, zeros
from src.modeling.force_model import c_f_m


def nonlinear_eom_to_ss(aircraft, x_ss, u_ss, x_0, u_0, m, j, dx=0.1, du=0.1):
    """aircraft system linearization routine."""
    """return jacobians a, b wrt to x_ss and output matrices c, and d wrt u_ss."""
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
