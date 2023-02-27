"""
Sigma modification

In the presence of a constant input bias, the "standard" adaptive law will cause the
parameter estimate to grow unbounded. The use of sigma modification shown here prevents
this from happening, although it doesn't provide convergence of the tracking error to
zero.

See Stable Adaptive Systems Anuradha M. Annaswamy and Kumpati S. Narendra, 2005 (page 310)
"""

import control
import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
# input: [u, v]
# state: x_p
# output: x_p
A_P = 4
B_P = np.array([1, 1])
C_P = 1
D_P = 0

# Reference model parameters
# input: r
# state: x_m
# output: x_m
A_M = -1
B_M = 1
C_M = 1
D_M = 0

# Control parameters
SIGMA = 1
V = 10

# Initial conditions: error
THETA_TILDE_0 = 5
E_0 = 5

# More initial conditions: parameter, reference model, and plant states
THETA_STAR = A_M - A_P
THETA_0 = THETA_TILDE_0 + THETA_STAR
X_M_0 = 0
X_P_0 = E_0 + X_M_0

# Define plant
IO_PLANT = control.LinearIOSystem(
    control.StateSpace(A_P, B_P, C_P, D_P),
    inputs=('u', 'v'),
    outputs=('x_p'),
    states=('x_p'),
    name='plant'
)

# Define reference model
IO_REF_MODEL = control.LinearIOSystem(
    control.StateSpace(A_M, B_M, C_M, D_M),
    inputs=('r'),
    outputs=('x_m'),
    states=('x_m'),
    name='ref_model'
)

def adaptive_state(_t, x_state, u_input, _params):
    """Internal state of adpative controller"""

    # Controller inputs
    x_p = u_input[1]
    x_m = u_input[2]

    # Controller state
    theta = x_state[0]

    # Algebraic relationships
    e = x_p - x_m

    # Dynamics
    dot_theta = -e * x_p - SIGMA * theta

    return [dot_theta]

def adaptive_output(_t, x_state, u_input, _params):
    """Algebraic output from adaptive controller"""

    # Controller inputs
    r = u_input[0]
    x_p = u_input[1]
    x_m = u_input[2]

    # Controller state
    theta = x_state[0]

    # Algebraic relationships
    e = x_p - x_m

    # Control law
    u = theta * x_p + r

    # Constant bias
    v = V

    return [u, v, theta, e]

IO_ADAPTIVE = control.NonlinearIOSystem(
    adaptive_state,
    adaptive_output,
    inputs=3,
    outputs=('u', 'v', 'theta', 'e'),
    states=1,
    name='control',
    dt=0
)

# Define the closed-loop system
# x_cl = [plant.x_p, ref_model.x_m, control.theta]
IO_CLOSED = control.InterconnectedSystem(
    (IO_PLANT, IO_REF_MODEL, IO_ADAPTIVE),
    connections=(
        ('plant.u', 'control.u'),
        ('plant.v', 'control.v'),
        ('control.u[1]', 'plant.x_p'),
        ('control.u[2]', 'ref_model.x_m')
    ),
    inplist=('control.u[0]'),
    outlist=('plant.x_p', 'ref_model.x_m', 'control.u', 'control.v', 'control.theta', 'control.e'),
    dt=0
)

# Set simulation duration and time steps
N_POINTS = 3000
T_F = 30

# Set initial conditions
X0 = np.zeros((3, 1))
X0[0] = X_P_0
X0[1] = X_M_0
X0[2] = THETA_0

# Define simulation time span and control input
T = np.linspace(0, T_F, N_POINTS)
U = np.zeros((1, N_POINTS))
R_IN = np.zeros(N_POINTS)
U[0, :] = R_IN

# Simulate the system
T_OUT, Y_OUT = control.input_output_response(IO_CLOSED, T, U, X0)

# Plot the response
plt.rc('text', usetex=True)
plt.rc('font', family='sans')

FIG = plt.figure(1, figsize=(6, 6), dpi=300, facecolor='w', edgecolor='k')

BLUE = '#1269d3'
WHITE = '#ffffff'

AX_1 = FIG.add_subplot(3, 1, 1)
AX_1.plot(T_OUT, Y_OUT[0] - Y_OUT[1], label=r'$x_m$', color=BLUE)
AX_1.set_title('Tracking Error')
AX_1.set_ylabel(r'$e$')
AX_1.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_1.set_facecolor(WHITE)

AX_2 = FIG.add_subplot(3, 1, 2)
AX_2.plot(T_OUT, Y_OUT[4] - THETA_STAR, label=r'$\tilde{\theta}$', color=BLUE)
AX_2.set_title('Parameter Error')
AX_2.set_ylabel(r'$\tilde{\theta}$')
AX_2.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2.set_facecolor(WHITE)

AX_3 = FIG.add_subplot(3, 1, 3)
AX_3.plot(Y_OUT[5], Y_OUT[4] - THETA_STAR, label=r'$\tilde{\theta}$', color=BLUE)
AX_3.set_title('Errors')
AX_3.set_ylabel(r'$\tilde{\theta}$')
AX_3.set_xlabel(r'$e$', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_3.set_facecolor(WHITE)

FIG.tight_layout()
FIG.savefig('sigma_mod.png', bbox_inches='tight')
