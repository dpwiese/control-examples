"""
Classical MIMO Adaptive Control
"""

import control
import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
# input: u
# state: x_p
# output: x_p
A_P = np.array([
    [-2, -1, 0, 0, 0],
    [1, 0, 0, 0, 0],
    [0, 0, -4, -2, 0],
    [0, 0, 2, 0, 0],
    [0, 0, 0, 0, 3]
    ])
B_P = np.array([[1, 0], [0, 0], [0, 1], [0, 0], [0, 1]])
C_P = np.array([[1, 1, 0, 1, 0], [0, 1, 0, 0, 1]])
D_P = np.array([[0, 0], [0, 0]])

# Define plant
IO_PLANT = control.LinearIOSystem(
    control.StateSpace(A_P, B_P, C_P, D_P),
    inputs=2,
    outputs=2,
    states=5,
    name='plant'
)

# Stable Hermite form
A = 1

# # Reference Model
NUM_WM = [[np.array([0]) for i in range(2)] for j in range(2)]
DEN_WM = [[np.array([1]) for i in range(2)] for j in range(2)]

NUM_WM[0][0] = np.array([A])
DEN_WM[0][0] = np.array([1., A])
NUM_WM[1][1] = np.array([A])
DEN_WM[1][1] = np.array([1., A])

IO_REF_MODEL = control.LinearIOSystem(
    control.tf2ss(NUM_WM, DEN_WM),
    inputs=2,
    outputs=2,
    states=2,
    name='ref_model'
)

# Filter
R_Q = np.array([1., 1., 1.])

# Input filters
NUM_IN = [[np.array([0]) for j in range(2)] for i in range(4)]
DEN_IN = [[np.array([1]) for j in range(2)] for i in range(4)]

NUM_IN[0][0] = np.array([1.])
DEN_IN[0][0] = R_Q
NUM_IN[1][1] = np.array([1.])
DEN_IN[1][1] = R_Q

NUM_IN[2][0] = np.array([1., 0.])
DEN_IN[2][0] = R_Q
NUM_IN[3][1] = np.array([1., 0.])
DEN_IN[3][1] = R_Q

# Define input filters
# input: u
# outputs: omega_1, omega_2
IO_INPUT_FILTER = control.LinearIOSystem(
    control.tf2ss(NUM_IN, DEN_IN),
    inputs=2,
    outputs=4,
    states=4,
    name='input_filter'
)

# Output filters
NUM_OUT = [[np.array([0]) for j in range(2)] for i in range(6)]
DEN_OUT = [[np.array([1]) for j in range(2)] for i in range(6)]

NUM_OUT[0][0] = np.array([1.])
DEN_OUT[0][0] = R_Q
NUM_OUT[1][1] = np.array([1.])
DEN_OUT[1][1] = R_Q

NUM_OUT[2][0] = np.array([1., 0.])
DEN_OUT[2][0] = R_Q
NUM_OUT[3][1] = np.array([1., 0.])
DEN_OUT[3][1] = R_Q

NUM_OUT[4][0] = np.array([1., 0., 0.])
DEN_OUT[4][0] = R_Q
NUM_OUT[5][1] = np.array([1., 0., 0.])
DEN_OUT[5][1] = R_Q

# Define output filters
# input: y_p
# outputs: omega_3, omega_4, omega_5
IO_OUTPUT_FILTER = control.LinearIOSystem(
    control.tf2ss(NUM_OUT, DEN_OUT),
    inputs=2,
    outputs=6,
    states=4,
    name='output_filter'
)

def adaptive_state(_t, x_state, u_input, _params):
    """Internal state of adpative controller"""

    # Algebraic Relationships: error: e_1 = y_p - y_m
    e_1_1 = u_input[12] - u_input[14]
    e_1_2 = u_input[13] - u_input[15]

    # Dynamics: update laws
    return [
        -e_1_1 * u_input[0],
        -e_1_2 * u_input[1],
        -e_1_1 * u_input[2],
        -e_1_2 * u_input[3],
        -e_1_1 * u_input[4],
        -e_1_2 * u_input[5],
        -e_1_1 * u_input[6],
        -e_1_2 * u_input[7],
        -e_1_1 * u_input[8],
        -e_1_2 * u_input[9],
        -e_1_1 * u_input[10],
        -e_1_2 * u_input[11]
        ]

def adaptive_output(_t, x_state, u_input, _params):
    """Algebraic output from adaptive controller"""

    # Define Theta and omega
    theta_1 = [x_state[0], x_state[2], x_state[4], x_state[6], x_state[8], x_state[10]]
    theta_2 = [x_state[1], x_state[3], x_state[5], x_state[7], x_state[9], x_state[11]]
    omega_1 = [u_input[0], u_input[2], u_input[4], u_input[6], u_input[8], u_input[10]]
    omega_2 = [u_input[1], u_input[3], u_input[5], u_input[7], u_input[9], u_input[11]]

    # Control law
    # u = Theta * omega
    return [
        sum([a*b for a, b in zip(theta_1, omega_1)]),
        sum([a*b for a, b in zip(theta_2, omega_2)])
        ]

IO_ADAPTIVE = control.NonlinearIOSystem(
    adaptive_state,
    adaptive_output,
    inputs=16,
    outputs=2,
    states=12,
    name='control',
    dt=0
)

# Define the closed-loop system
# x_cl = [plant[5], reference[2], input-filter[4], output_filter[4], controller[12]]
IO_CLOSED = control.InterconnectedSystem(
    (IO_PLANT, IO_REF_MODEL, IO_INPUT_FILTER, IO_OUTPUT_FILTER, IO_ADAPTIVE),
    connections=(
        ('plant.u[0]', 'control.y[0]'),
        ('plant.u[1]', 'control.y[1]'),
        ('input_filter.u[0]', 'control.y[0]'),
        ('input_filter.u[1]', 'control.y[1]'),
        ('output_filter.u[0]', 'plant.y[0]'),
        ('output_filter.u[1]', 'plant.y[1]'),
        ('control.u[2]', 'input_filter.y[0]'),
        ('control.u[3]', 'input_filter.y[1]'),
        ('control.u[4]', 'input_filter.y[2]'),
        ('control.u[5]', 'input_filter.y[3]'),
        ('control.u[6]', 'output_filter.y[0]'),
        ('control.u[7]', 'output_filter.y[1]'),
        ('control.u[8]', 'output_filter.y[2]'),
        ('control.u[9]', 'output_filter.y[3]'),
        ('control.u[10]', 'output_filter.y[4]'),
        ('control.u[11]', 'output_filter.y[5]'),
        ('control.u[12]', 'plant.y[0]'),
        ('control.u[13]', 'plant.y[1]'),
        ('control.u[14]', 'ref_model.y[0]'),
        ('control.u[15]', 'ref_model.y[1]')
    ),
    inplist=('ref_model.u[0]', 'ref_model.u[1]', 'control.u[0]', 'control.u[1]'),
    outlist=('plant.y[0]', 'plant.y[1]', 'ref_model.y[0]', 'ref_model.y[1]'),
    dt=0
)

# Set simulation duration and time steps
N_POINTS = 6000
T_F = 120

# Set initial conditions
X0 = np.zeros((27, 1))

# Define simulation time span and control input
T = np.linspace(0, T_F, N_POINTS)
U = np.zeros((4, N_POINTS))

R_IN_1 = np.zeros(N_POINTS)
R_IN_1[1000:2000] = 1
R_IN_1[3000:4000] = 1
R_IN_1[5000:6000] = 1

R_IN_2 = np.zeros(N_POINTS)
R_IN_2[1000:2000] = -1
R_IN_2[3000:4000] = -1
R_IN_2[5000:6000] = -1

U[0, :] = R_IN_1
U[1, :] = R_IN_2
U[2, :] = R_IN_1
U[3, :] = R_IN_2

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


AX_1_1 = FIG_1.add_subplot(2, 1, 1)
AX_1_1.plot(T_OUT, R_IN_1, label=r'$r_{1}$', color=RED)
AX_1_1.plot(T_OUT, Y_OUT[2], label=r'$y_{m_{1}}$', color=BLUE)
AX_1_1.plot(T_OUT, Y_OUT[0], label=r'$y_{p_{1}}$', color=GREEN)
AX_1_1.set_title('Plant and Reference Model Outputs 1')
AX_1_1.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_1_1.legend(loc="upper right", bbox_to_anchor=(1, 1), fontsize=9)
AX_1_1.set_facecolor(WHITE)

AX_1_2 = FIG_1.add_subplot(2, 1, 2)
AX_1_2.plot(T_OUT, R_IN_2, label=r'$r_{2}$', color=RED)
AX_1_2.plot(T_OUT, Y_OUT[3], label=r'$y_{m_{2}}$', color=BLUE)
AX_1_2.plot(T_OUT, Y_OUT[1], label=r'$y_{p_{2}}$', color=GREEN)
AX_1_2.set_title('Plant and Reference Model Output 2')
AX_1_2.set_xlabel(r'time ($t$)', fontname="Times New Roman", fontsize=9, fontweight=100)
AX_1_2.legend(loc="upper right", bbox_to_anchor=(1, 1), fontsize=9)
AX_1_2.set_facecolor(WHITE)

# Format figures
FIG_1.tight_layout(rect=[0, 0.03, 1, 0.95])
# FIG_2.tight_layout(rect=[0, 0.03, 1, 0.95])

# Save figures
FIG_1.savefig('classical_mimo_output.png', bbox_inches='tight')
