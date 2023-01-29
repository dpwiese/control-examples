"""
Boeing 747-100 linear model
Flight condition: M = 0.9, h = 40,000ft
Dimensional derivatives taken from NASA report CR-2144 (page 230, 234)
Also see Nelson Appendix B Table B.27 page 416
"""

import control
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

# Gravity [ft/s^2]
G = 32.3

# Equlibrium flight condition [ft/s]
MACH = 0.9
U_EQ = 871
GAMMA_EQ = 0
W_EQ = 0
ALPHA_EQ = 0
V_EQ = U_EQ

# Weight and other parameters
# Moments of inertia [slug-ft^2]
J_XX = 1.82 * 10**7
J_YY = 3.31 * 10**7
J_ZZ = 4.97 * 10**7
J_XZ = 9.70 * 10**5

# Weight and planform [lb], [ft^2]
WEIGHT = 636600
S_WING = 5500

# Longitudinal stability derivatives
# X
X_U = -0.0200
X_W = 0.0159
X_Q = 0
X_W_DOT = 0
X_D_TH = 0.505 * 10**-4
X_D_E = 0.781

# Z
Z_U = -0.0424
Z_W = -0.401
Z_Q = -6.71
Z_W_DOT = 0.00614
Z_D_TH = -0.220 * 10**-5
Z_D_E = -18.6

# M
M_U = -0.623 * 10**-4
M_W = -0.00190
M_Q = -0.401
M_W_DOT = -0.000160
M_D_TH = 0.302 * 10**-6
M_D_E = -1.22

# Calculate
# See Nelson Eq. 4.73 and 4.74
X_V = X_U
Z_V = Z_U
M_V = M_U
X_ALPHA = X_W * V_EQ
Z_ALPHA = Z_W * V_EQ
M_ALPHA = M_W * V_EQ
M_ALPHA_DOT = M_W_DOT * V_EQ

# Lateral stability derivatives
# Y
Y_V = -0.0605
Y_P = 0
Y_R = 0
Y_D_A = 0
Y_D_R = 4.0380

# L
L_V = -0.0016
L_P = -0.4592
L_R = 0.2875
L_D_A = -0.1863
L_D_R = 0.1236

# N
N_V = 0.0011
N_P = -0.0118
N_R = -0.1465
N_D_A = 0.0097
N_D_R = -0.4439

# Longitudinal matrices
E_P_LONG = np.array([
    [1, 0, 0, 0],
    [0, 1 - Z_W_DOT, 0, 0],
    [0, -M_W_DOT, 1, 0],
    [0, 0, 0,1]])

A_P_PRIME_LONG = np.array([
    [X_U, X_W, X_Q - W_EQ, -G * np.cos(GAMMA_EQ)],
    [Z_U, Z_W, Z_Q + U_EQ, -G * np.sin(GAMMA_EQ)],
    [M_U, M_W, M_Q, 0],
    [0, 0, 1, 0]])

B_P_PRIME_LONG = np.array([
    [X_D_TH, X_D_E],
    [Z_D_TH, Z_D_E],
    [M_D_TH, M_D_E],
    [0, 0]])

# Lateral-directional matrices
E_P_LATR = np.array([
    [1, 0, 0, 0],
    [0, 1, -J_XZ / J_XX, 0],
    [0, -J_XZ / J_ZZ, 1, 0],
    [0, 0, 0, 1]])

A_P_PRIME_LATR = np.array([
    [Y_V, Y_P, Y_R - U_EQ, -G * np.cos(GAMMA_EQ)],
    [L_V, L_P, L_R, 0],
    [N_V, N_P, N_R, 0],
    [0, 1, 0, 0]])

B_P_PRIME_LATR = np.array([
    [Y_D_A, Y_D_R],
    [L_D_A, L_D_R],
    [N_D_A, N_D_R],
    [0, 0]])

def boeing_747_linear_longitudinal_body_4():
    """
    Boeing 747 longitudinal dynamics
    Input: [throttle, elevator]
    State: [u, w, q, theta]
    Output: [u, w, q, theta]
    This is the basic longitudinal dynamics in the form:
    E \dot{x} = A^{prime}x + B^{prime}u
    """

    e_p = E_P_LONG
    a_p_prime = A_P_PRIME_LONG
    b_p_prime = B_P_PRIME_LONG

    a_p = np.matmul(inv(e_p), a_p_prime)
    b_p = np.matmul(inv(e_p), b_p_prime)
    c_p = np.identity(4)
    d_p = np.zeros((4,2))

    c_pz = np.array([[0, 0, 1, 0]])
    d_pz = np.zeros((1,2))

    return [a_p, b_p, c_p, d_p, c_pz, d_pz]

def boeing_747_linear_longitudinal_body_5a():
    """
    Boeing 747 longitudinal dynamics
    Input: [throttle, elevator]
    State: [u, w, q, theta, h]
    Output: [u, w, q, theta, h]
    This extends the longitudinal dynamics in the form:
    E \dot{x} = A^{prime}x + B^{prime}u
    To include altitude as a state given by:
    \dot{h} = u_{eq} (\theta - w)
    """

    e_p_row_1 = np.concatenate((E_P_LONG, np.array([[0, 0, 0, 0]]).T), axis=1)
    e_p_row_2 = np.array([[0, 0, 0, 0, 1]])
    e_p = np.concatenate((e_p_row_1, e_p_row_2), axis=0)

    a_p_prime_row_1 = np.concatenate((A_P_PRIME_LONG, np.array([[0, 0, 0, 0]]).T), axis=1)
    a_p_prime_row_2 = np.array([[0, -1, 0, U_EQ, 0]])
    a_p_prime = np.concatenate((a_p_prime_row_1, a_p_prime_row_2), axis=0)

    b_p_prime =np.concatenate((B_P_PRIME_LONG, np.array([[0, 0]])), axis=0)

    a_p = np.matmul(inv(e_p), a_p_prime)
    b_p = np.matmul(inv(e_p), b_p_prime)
    c_p = np.identity(5)
    d_p = np.zeros((5,2))

    c_pz = np.array([[0, 0, 1, 0, 0]])
    d_pz = np.zeros((1,2))

    return [a_p, b_p, c_p, d_p, c_pz, d_pz]

def boeing_747_linear_longitudinal_body_5b():
    """
    Boeing 747 longitudinal dynamics
    Input: [throttle, elevator]
    State: [u, w, q, theta, h]
    Output: [u, w, q, theta, h]
    This is the 5-state longitudinal system above in the form:
    \dot{x} = Ax + Bu
    Having performed the left-multiplication by E^{-1} and simplifying some additional terms
    This expression is consistent with, for example, Nelson Eq. 4.51
    """

    a_p = np.array([
        [X_U, X_W, 0, -G, 0],
        [Z_U, Z_W, U_EQ, 0, 0],
        [M_U + M_W_DOT * Z_U, M_W + M_W_DOT * Z_W, M_Q + M_W_DOT * U_EQ, 0, 0],
        [0, 0, 1, 0, 0],
        [0, -1, 0, U_EQ, 0]])

    b_p = np.array([
        [X_D_TH, X_D_E],
        [Z_D_TH, Z_D_E],
        [M_D_TH + M_W_DOT * Z_D_TH, M_D_E + M_W_DOT * Z_D_E],
        [0, 0],
        [0, 0]])

    c_p = np.identity(5)
    d_p = np.zeros((5,2))

    c_pz = np.array([
        [1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]])
    d_pz = np.array([[0], [0], [0], [0]])

    return [a_p, b_p, c_p, d_p, c_pz, d_pz]

def boeing_747_linear_longitudinal_stability_5():
    """
    Boeing 747 longitudinal dynamics
    Input: [throttle, elevator]
    State: [V, alpha, q, theta, h]
    Output: [V, alpha, q, theta, h]
    See Lavretsky, Wise Eq. 1.17, 1.18 and Nelson 4.75
    """

    a_p = np.array([
        [X_V, X_ALPHA, 0, -G * np.cos(GAMMA_EQ), 0],
        [Z_V / V_EQ, Z_ALPHA / V_EQ, 1 + Z_Q / V_EQ, - G * np.sin(GAMMA_EQ) / V_EQ, 0],
        [M_V, M_ALPHA + M_ALPHA_DOT * Z_ALPHA / V_EQ, M_Q + M_ALPHA_DOT, 0, 0],
        [0, 0, 1, 0, 0],
        [0, -V_EQ, 0, V_EQ, 0]])

    b_p = np.array([
        [X_D_TH * np.cos(ALPHA_EQ), X_D_E],
        [-X_D_TH * np.sin(ALPHA_EQ), Z_D_E / V_EQ],
        [M_D_TH, M_D_E],
        [0, 0],
        [0, 0]])

    c_p = np.identity(5)
    d_p = np.zeros((5,2))

    c_pz = np.array([
        [1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]])
    d_pz = np.zeros((4,2))

    return [a_p, b_p, c_p, d_p, c_pz, d_pz]

def boeing_747_linear_lateral_5():
    """
    Boeing 747 lateral-directional dynamics
    Input: [aileron, rudder]
    State: [v, p, r, phi, psi]
    Output: [v, p, r, phi, psi]
    See Nelson Eq. 5.33
    """

    e_latr = E_P_LATR
    a_latr_prime = A_P_PRIME_LATR
    b_latr_prime = B_P_PRIME_LATR

    a_p_latr = np.matmul(inv(e_latr), a_latr_prime)
    b_p_latr = np.matmul(inv(e_latr), b_latr_prime)

    a_p_row_1 = np.concatenate((a_p_latr, np.array([[0, 0, 0, 0]]).T), axis=1)
    a_p_row_2 = np.array([[0, 0, 1, 0, 0]])

    # Augment with heading angle
    a_p = np.concatenate((a_p_row_1, a_p_row_2), axis=0)
    b_p = np.concatenate((b_p_latr, np.zeros((1,2))), axis=0)
    c_p = np.identity(5)
    d_p = np.zeros((5,2))
    c_pz = np.array([0, 1, 0, 0, 0])
    d_pz = np.zeros((1,1))

    return [a_p, b_p, c_p, d_p, c_pz, d_pz]
