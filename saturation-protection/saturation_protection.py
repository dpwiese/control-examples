"""
Saturation protection

Adaptive Control in the Presence of Input Constraints
S.P. Karason ; A.M. Annaswamy
https://doi.org/10.23919/ACC.1993.4793095
https://doi.org/10.1109/9.333787
"""

import control
import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
# input: u
# state: x_p
# output: x_p
A_P = 1
B_P = 1
C_P = 1
D_P = 0

# Reference model parameters
# input: r
# state: x_m
# output: x_m
K_M = 1
A_M = -1
B_M = 1
C_M = 1
D_M = 0

# Initial conditions: plant and reference model
X_M_0 = -9.9
X_P_0 = -9.99

# Initial conditions: controller
THETA_0 = -2.2
K_0 = 1.1
BETA_DELTA_0 = 0
E_DELTA_0 = 0

# Actuator saturation limit
U_MAX = 10

# Gains
GAMMA_1 = 0.1
GAMMA_2 = 0.1
GAMMA_3 = 1

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
    control.StateSpace(A_M, B_M, C_M, D_M),
    inputs=('r'),
    outputs=('x_m'),
    states=('x_m'),
    name='ref_model'
)

def adaptive_state(_t, x_state, u_input, _params): # pylint: disable=too-many-locals
    """Internal state of adpative controller"""

    # Controller inputs
    r = u_input[0]
    x_p = u_input[1]
    x_m = u_input[2]

    # Controller state
    theta = x_state[0]
    k = x_state[1]
    beta_delta = x_state[2]
    e_delta = x_state[3]

    # Algebraic relationships
    e = x_p - x_m
    e_u = e - e_delta

    # Control law
    u_c = theta * x_p + k * r
    u = u_c if abs(u_c) <= U_MAX else U_MAX * np.sign(u_c)
    delta_u = u - u_c

    # Dynamics: update laws
    dot_theta = -GAMMA_1 * np.sign(B_P) * e_u * x_p
    dot_k = -GAMMA_2 * np.sign(B_P) * e_u * r
    dot_beta_delta = GAMMA_3 * e_u * delta_u

    # Dynamics: error state
    dot_e_delta = A_M * e_delta + beta_delta * delta_u

    # # Uncomment me to revert to standard adaptive system
    # dot_theta = -GAMMA_1 * np.sign(B_P) * e * x_p
    # dot_k = -GAMMA_2 * np.sign(B_P) * e * r
    # dot_beta_delta = 0
    # dot_e_delta = 0

    return [dot_theta, dot_k, dot_beta_delta, dot_e_delta]

def adaptive_output(_t, x_state, u_input, _params):
    """Algebraic output from adaptive controller"""

    # Controller inputs
    r = u_input[0]
    x_p = u_input[1]
    x_m = u_input[2]

    # Controller state
    theta = x_state[0]
    k = x_state[1]
    e_delta = x_state[3]

    # Algebraic relationships
    e = x_p - x_m
    e_u = e - e_delta

    # Control law
    u_c = theta * x_p + k * r
    u = u_c if abs(u_c) <= U_MAX else U_MAX * np.sign(u_c)
    delta_u = u - u_c

    return [u, theta, k, e, delta_u, u_c, e_delta, e_u]

IO_ADAPTIVE = control.NonlinearIOSystem(
    adaptive_state,
    adaptive_output,
    inputs=3,
    outputs=('u', 'theta', 'k', 'e', 'delta_u', 'u_c', 'e_delta', 'e_u'),
    states=4,
    name='control',
    dt=0
)

# Define the closed-loop system
# x_cl = [plant.x_p, ref_model.x_m, control.theta, control.k, control.beta_delta, control.e_delta]
IO_CLOSED = control.InterconnectedSystem(
    (IO_PLANT, IO_REF_MODEL, IO_ADAPTIVE),
    connections=(
        ('plant.u', 'control.u'),
        ('control.u[1]', 'plant.x_p'),
        ('control.u[2]', 'ref_model.x_m')
    ),
    inplist=('ref_model.r', 'control.u[0]'),
    outlist=('plant.x_p',
             'ref_model.x_m',
             'control.u',
             'control.theta',
             'control.k',
             'control.e',
             'control.u_c',
             'control.e_delta',
             'control.e_u'
            ),
    dt=0
)

# Set simulation duration and time steps
N_POINTS = 3000
T_F = 20

# Set initial conditions
X0 = np.zeros((6, 1))
X0[0] = X_P_0
X0[1] = X_M_0
X0[2] = THETA_0
X0[3] = K_0
X0[4] = BETA_DELTA_0
X0[5] = E_DELTA_0

# Define simulation time span and control input
T = np.linspace(0, T_F, N_POINTS)
U = np.zeros((2, N_POINTS))
R_IN = 5 * np.sin(0.5 * T)
U[0, :] = R_IN
U[1, :] = R_IN

# Simulate the system
T_OUT, Y_OUT = control.input_output_response(IO_CLOSED, T, U, X0)

# Plot the response
plt.rc('text', usetex=True)
plt.rc('font', family='sans')

# Define plot styles
RED = '#f62d73'
BLUE = '#1269d3'
WHITE = '#ffffff'
GREEN = '#2df643'

# Make figures
FIG_1 = plt.figure(1, figsize=(6, 6), dpi=300, facecolor='w', edgecolor='k')
FIG_2 = plt.figure(2, figsize=(6, 6), dpi=300, facecolor='w', edgecolor='k')

# Populate figure 1
FIG_1.suptitle(
    r'$\gamma_{1} = $' + str(GAMMA_1) +
    r'$, \gamma_{2} = $' + str(GAMMA_2) +
    r'$, \gamma_{3} = $' + str(GAMMA_3), fontsize=12
    )

AX_1_1 = FIG_1.add_subplot(2, 1, 1)
AX_1_1.plot(T_OUT, Y_OUT[1], label=r'$x_m$', color=BLUE)
AX_1_1.plot(T_OUT, Y_OUT[0], label=r'$x_p$', color=GREEN)
AX_1_1.set_title('Plant and Reference Model States')
AX_1_1.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_1_1.legend(loc="lower right", bbox_to_anchor=(1, 0), fontsize=9)
AX_1_1.set_facecolor(WHITE)
AX_1_1.set_ylim([-12, 12])

AX_1_2 = FIG_1.add_subplot(2, 1, 2)
AX_1_2.plot(T_OUT, Y_OUT[2], label=r'$u$', color=BLUE)
AX_1_2.plot(T_OUT, Y_OUT[6], label=r'$u_c$', color=GREEN)
AX_1_2.set_title('Control Effort')
AX_1_2.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_1_2.legend(loc="lower right", bbox_to_anchor=(1, 0), fontsize=9)
AX_1_2.set_facecolor(WHITE)
AX_1_2.set_ylim([-10, 30])

# Populate figure 2
AX_2_1 = FIG_2.add_subplot(3, 1, 1)
AX_2_1.plot(T_OUT, Y_OUT[5], label=r'$e$', color=BLUE)
AX_2_1.set_title('Tracking Error')
AX_2_1.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2_1.set_ylabel(r'$e$', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2_1.set_facecolor(WHITE)
AX_2_1.set_ylim([-20, 20])

AX_2_2 = FIG_2.add_subplot(3, 1, 2)
AX_2_2.plot(T_OUT, Y_OUT[7], label=r'$e_{\delta}$', color=BLUE)
AX_2_2.set_title('Deficit Error')
AX_2_2.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2_2.set_ylabel(r'$e_{\Delta}$', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2_2.set_facecolor(WHITE)
AX_2_2.set_ylim([-20, 20])

AX_2_3 = FIG_2.add_subplot(3, 1, 3)
AX_2_3.plot(T_OUT, Y_OUT[8], label=r'$e_{u}$', color=BLUE)
AX_2_3.set_title('Controllable Error')
AX_2_3.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2_3.set_ylabel(r'$e_{u}$', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_2_3.set_facecolor(WHITE)
AX_2_3.set_ylim([-2, 2])

# Format figures
FIG_1.tight_layout(rect=[0, 0.03, 1, 0.95])
FIG_2.tight_layout(rect=[0, 0.03, 1, 0.95])

# Save figures
FIG_1.savefig('saturation_protection_state_input.png', bbox_inches='tight')
FIG_2.savefig('saturation_protection_errors.png', bbox_inches='tight')
