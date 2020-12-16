"""
This example shows reference model adaptive control of a simple, scalar system.
A closed-loop reference model is used, and the tuning gain and CRM gain can be
varied and the resulting closed-loop performance to a simple step command.
"""

import control
import numpy as np
import matplotlib.pyplot as plt

# Gains
L = -100
GAMMA = 100

# Plant parameters
# input: u
# state: x_p
# output: x_p
A_P = 1
B_P = 2
C_P = 1
D_P = 0

# Reference model parameters
# input: [r, x_p]
# state: x_m
# output: x_m
K_M = 1
A_M = -1
B_M = np.array([K_M, -L])
C_M = 1
D_M = 0

# Initial conditions
X_M_0 = 0
X_P_0 = 0
THETA_0 = 0
K_0 = 0

# Define plant
IO_PLANT = control.LinearIOSystem(
    control.StateSpace(A_P, B_P, C_P, D_P),
    inputs=('u'),
    outputs=('x_p'),
    states=('x_p'),
    name='plant'
)

# Define reference model
IO_REF_MODEL = control.LinearIOSystem(
    control.StateSpace(A_M + L, B_M, C_M, D_M),
    inputs=('r', 'x_p'),
    outputs=('x_m'),
    states=('x_m'),
    name='ref_model'
)

def adaptive_state(_t, x_state, u_input, _params):
    """Internal state of adpative controller"""

    # Controller inputs
    r = u_input[0]
    x_p = u_input[1]
    x_m = u_input[2]

    # Algebraic relationships
    e = x_p - x_m

    # Dynamics
    dot_theta = -GAMMA * np.sign(B_P) * e * x_p
    dot_k = -GAMMA * np.sign(B_P) * e * r

    return [dot_theta, dot_k]

def adaptive_output(_t, x_state, u_input, _params):
    """Algebraic output from adaptive controller"""

    # Controller inputs
    r = u_input[0]
    x_p = u_input[1]
    x_m = u_input[2]

    # Controller state
    theta = x_state[0]
    k = x_state[1]

    # Algebraic relationships
    e = x_p - x_m

    # Control law
    u = theta * x_p + k * r

    return [u, theta, k, e]

IO_ADAPTIVE = control.NonlinearIOSystem(
    adaptive_state,
    adaptive_output,
    inputs=3,
    outputs=('u', 'theta', 'k', 'e'),
    states=2,
    name='control',
    dt=0
)

# Define the closed-loop system
# x_cl = [plant.x_p, ref_model.x_m, control.theta, control.k]
IO_CLOSED = control.InterconnectedSystem(
    (IO_PLANT, IO_REF_MODEL, IO_ADAPTIVE),
    connections=(
        ('plant.u', 'control.u'),
        ('ref_model.x_p', 'plant.x_p'),
        ('control.u[1]', 'plant.x_p'),
        ('control.u[2]', 'ref_model.x_m')
    ),
    inplist=('ref_model.r', 'control.u[0]'),
    outlist=('plant.x_p', 'ref_model.x_m', 'control.u', 'control.theta', 'control.k', 'control.e'),
    dt=0
)

# Set simulation duration and time steps
N_POINTS = 3000
T_F = 30

# Set initial conditions
X0 = np.zeros((4, 1))
X0[0] = X_P_0
X0[1] = X_M_0
X0[2] = THETA_0
X0[3] = K_0

# Define simulation time span and control input
T = np.linspace(0, T_F, N_POINTS)
U = np.zeros((2, N_POINTS))
R_IN = np.zeros(N_POINTS)
R_IN[500:N_POINTS] = 1
U[0, :] = R_IN
U[1, :] = R_IN

# Simulate the system
T_OUT, Y_OUT = control.input_output_response(IO_CLOSED, T, U, X0)

# Plot the response
plt.rc('text', usetex=True)
plt.rc('font', family='sans')

FIG = plt.figure(1, figsize=(6, 3), dpi=300, facecolor='w', edgecolor='k')
FIG.suptitle(r'$\gamma = $' + str(GAMMA) + r'$, \ell = $' + str(L), fontsize=12)

RED = '#f62d73'
BLUE = '#1269d3'
WHITE = '#ffffff'
GREEN = '#2df643'

AX_1 = FIG.add_subplot(1, 2, 1)
AX_1.plot(T_OUT, R_IN, label=r'$r$', color=RED)
AX_1.plot(T_OUT, Y_OUT[1], label=r'$x_m$', color=BLUE)
AX_1.plot(T_OUT, Y_OUT[0], label=r'$x_p$', color=GREEN)
AX_1.set_title('Plant and Reference Model States')
AX_1.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_1.legend(loc="lower right", bbox_to_anchor=(1, 0), fontsize=9)
AX_1.set_facecolor(WHITE)

AX_2 = FIG.add_subplot(1, 2, 2)
AX_2.plot(T_OUT, Y_OUT[3], label=r'$\theta$', color=BLUE)
AX_2.plot(T_OUT, Y_OUT[4], label=r'$k$', color=GREEN)
AX_2.set_title('Parameter Estimates')
AX_2.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2.legend(loc="lower right", bbox_to_anchor=(1, 0), fontsize=9)
AX_2.set_facecolor(WHITE)

FIG.tight_layout(rect=[0, 0.03, 1, 0.95])
FIG.savefig('fig/orm_versus_crm_gamma_' + str(GAMMA) + '_ell_' + str(-L) + '.png', bbox_inches='tight')
