# """real time state-space simulator attempt, try building code in azure."""
# from src.airplanes.example.plane import plane
# from common import Gravity
# from control import StateSpace
# from common.equations_of_motion import nonlinear_eom_to_ss
# from src.analysis.trim import trim_alpha_de
# import matplotlib.pyplot as plt
# from numpy import array, zeros, deg2rad, cos, sin, pi
# import pygame
# import time
#
# k_pitch = 0.2
# k_roll = 0.1
#
# aircraft = plane
# altitude = 30000  # ft
# g = Gravity(altitude).gravity()  # f/s2
# v = 500  # ft/s
# dt = 0.01  # s
# throttle = 1
# trim_out = trim_alpha_de(aircraft, v, altitude, 0)
# aoa = deg2rad(trim_out[0])  # rad
# u = v*cos(aoa)  # ft/s
# w = v*sin(aoa)  # ft/s
# de = deg2rad(trim_out[1])  # rad
# n_axis = 1
# j = aircraft['weight']['inertia']
# m = aircraft['weight']['weight']/g  # slug
# cg = aircraft['weight']['cg']
# x = array([float(u), 0, float(w), 0, float(aoa), 0, 0, 0, 0, 0, 0, altitude])
# # x = [u v w phi theta psi p q r p_n p_e h]
# x_ss = [0, 1, 2, 3, 4, 5, 6, 7, 8]
# a, b, c, d = nonlinear_eom_to_ss(aircraft, x_ss, [0, 1, 2], x, array([0, float(de), 0, throttle]), m, j)
#
# sys = StateSpace(a, b, c, d)
# sys.A = array(sys.A)
# sys.B = array(sys.B)
# sys.C = array(sys.C)
# sys.D = array(sys.D)
# pygame.init()
# # Loop until the user clicks the close button.
# done = False
#
# # Initialize the joysticks.
# pygame.joystick.init()
#
# # -------- Main Program Loop -----------
# in_tf = []
# t = []
# theta = []
# phi = []
# pitch_command = []
# roll_command = []
# u = []
# axis = [0, 0]
# x = zeros((sys.A.shape[0], 1))
# t_0 = time.time()
# # plt.style.use('fivethirtyeight')
# joystick = pygame.joystick.Joystick(0)
# joystick.init()
# plt.figure(figsize=(8, 7))
#
# while not done:
#     t.append(time.time() - t_0)
#     for event in pygame.event.get():  # User did something.
#         if event.type == pygame.QUIT:  # If user clicked close.
#             done = True  # Flag that we are done so we exit this loop.
#         elif event.type == pygame.JOYBUTTONDOWN:
#             print("Joystick button pressed.")
#         elif event.type == pygame.JOYBUTTONUP:
#             print("Joystick button released.")
#
#     axis[0] = joystick.get_axis(0)
#     axis[1] = joystick.get_axis(1)
#     pitch_command.append(-axis[1] * k_pitch)
#     roll_command.append(axis[0] * k_roll)
#
#     if len(t) == 1:
#         theta.append(0)
#         phi.append(0)
#     else:
#         u = array([[roll_command[-1]], [pitch_command[-1]], [0]])
#         dxdt = sys.A @ x + sys.B @ u
#         dt = t[-1] - t[-2]
#         x = x + dxdt * dt
#         y = sys.C @ x + sys.D @ u
#         theta.append(x[4]*180/pi)
#         phi.append(x[3]*180/pi)
#
#     plt.subplot(2, 1, 1)
#     plt.title('Blue=Pitch, Red=Roll')
#     plt.plot(t, theta, 'b')
#     plt.plot(t, phi, 'r')
#     plt.plot([0, 500], [5, 5], 'g--', linewidth=1)
#     plt.plot([0, 500], [7, 7], 'g--', linewidth=1)
#     plt.ylim((-15, 15))
#     plt.xlim((t[-1]-10, t[-1]+10))
#     plt.ylabel('Degrees')
#
#     plt.subplot(2, 1, 2)
#     plt.plot(t, pitch_command, 'b', label='pitch')
#     plt.plot(t, roll_command, 'r', label='roll')
#     plt.ylim((-0.5, 0.5))
#     plt.xlim((t[-1]-10, t[-1]+10))
#     plt.ylabel('Controls')
#     plt.xlabel('time (sec)')
#     plt.pause(0.000001)
#
# pygame.quit()
