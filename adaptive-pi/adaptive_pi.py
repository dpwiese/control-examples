"""
This example applies the better adaptive controller to a DC motor with unknown moment
of inertia J and damping B. The plant is represented by the following stable, first
order linear system of relative degree one, with input u and output x:

G_p(s) = 1 / (Js + B)

The desired output is x_d. The adaptive PI control and update laws are given by:

u = J_hat * e_1 + B_hat * x + K * e_2
J_hat_dot = gamma_1 * e_2 * e_1
B_hat_dot = gamma_2 * e2 * x

The errors are given by:

e = x_d − x
e_1 = x_d_dot + lambda * e
e_2_dot = e_dot + lambda * e

With the controllers error integrator state and two parameter estimation states, the
closed loop system has four states. The closed-loop system inputs are the desired
output command x_d and its derivative x_d_dot. The plant state x is the output.
"""

import control
import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
J = 2
B = 0.5

# Control parameters
K = 1
LAMBDA = 1
GAMMA_1 = 1
GAMMA_2 = 1

# Initial parameter estimates
J_HAT_0 = 0.1
B_HAT_0 = 2

# Flag to disable adaptive control
IS_ADAPTIVE = 1

# Plant
# u_input: u
# x_state: x
# y_output: x
SS_PLANT = control.StateSpace(-B/J, 1/J, 1, 0)

IO_DC_MOTOR = control.LinearIOSystem(
    SS_PLANT,
    inputs=('u'),
    outputs=('x'),
    states=('x'),
    name='plant'
)

def adaptive_pi_state(_t, x_state, u_input, _params):
    """Internal state of adpative PI controller"""

    # Controller inputs
    x_d = u_input[0]
    x_d_dot = u_input[1]
    x = u_input[2]

    # Controller state
    e_i = x_state[2]

    # Algebraic relationships
    e = x_d - x
    e_1 = x_d_dot + LAMBDA * e
    e_2 = e + LAMBDA * e_i

    # Dynamics
    d_j_hat = GAMMA_1 * e_2 * e_1 * IS_ADAPTIVE
    d_b_hat = GAMMA_2 * e_2 * x * IS_ADAPTIVE
    e_i_dot = e

    return [d_j_hat, d_b_hat, e_i_dot]

def adaptive_pi_output(_t, x_state, u_input, _params):
    """Algebraic output from adaptive PI controller"""

    # Controller inputs
    x_d = u_input[0]
    x_d_dot = u_input[1]
    x = u_input[2]

    # Controller state
    j_hat = x_state[0]
    b_hat = x_state[1]
    e_i = x_state[2]

    # Algebraic relationships
    e = x_d - x
    e_1 = x_d_dot + LAMBDA * e
    e_2 = e + LAMBDA * e_i

    # Control law
    u = j_hat * e_1 + b_hat * x + K * e_2

    return [u, x_state[0], x_state[1]]

IO_ADAPTIVE_PI = control.NonlinearIOSystem(
    adaptive_pi_state,
    adaptive_pi_output,
    inputs=3,
    outputs=('u', 'j_hat', 'b_hat'),
    states=3,
    name='control',
    dt=0
)

# Define the closed-loop system
# x_cl = [plant.x, control.x[0], control.x[1], control.x[2]]
#      = [motorx, j_hat, b_hat, e_i]
# u_cl = [xd, xddot]
# y_cl = [motorx, Jhat, Bhat]
IO_CLOSED = control.InterconnectedSystem(
    (IO_DC_MOTOR, IO_ADAPTIVE_PI),
    connections=(
        ('plant.u', 'control.u'),
        ('control.u[2]', 'plant.x')
    ),
    inplist=('control.u[0]', 'control.u[1]'),
    outlist=('plant.x', 'control.j_hat', 'control.b_hat'),
    dt=0
)

# Set simulation duration and time steps
N_POINTS = 3000
T_F = 60

# Set initial conditions
X0 = np.zeros((4, 1))
X0[0] = 1
X0[1] = J_HAT_0
X0[2] = B_HAT_0

# Define simulation time span and control input
T = np.linspace(0, T_F, N_POINTS)
U = np.zeros((2, N_POINTS))
XD_IN = np.sin(0.2 * np.pi * T)
XD_DOT_IN = np.cos(0.2 * np.pi * T)
U[0, :] = XD_IN
U[1, :] = XD_DOT_IN

# Simulate the system
T_OUT, Y_OUT = control.input_output_response(IO_CLOSED, T, U, X0)

# Plot the response
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(T, XD_IN)
plt.plot(T_OUT, Y_OUT[0])
plt.legend(['x_d', 'x'])
plt.subplot(2, 1, 2)
plt.plot(T_OUT, Y_OUT[1])
plt.plot(T_OUT, Y_OUT[2])
plt.legend(['j_hat', 'b_hat'])
plt.show(block=True)
