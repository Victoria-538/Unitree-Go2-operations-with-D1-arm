import numpy as np
import sympy as sym
from sympy import pprint

import matplotlib.pyplot as plt
from pytransform3d import transformations as pt
from pytransform3d.plot_utils import plot_vector

# ==========================================
# 1. KINEMATIC MATRIX DEFINITIONS
# ==========================================
def DH_matrix(alpha, a, d, theta):
# DH transformation matrix using the convention 
#  T(i-1,i) = Rz(theta) Tz(d) Tx(a) Rx(alpha)

    ca = sym.cos(alpha)
    sa = sym.sin(alpha)

    ct = sym.cos(theta)
    st = sym.sin(theta)

    return sym.Matrix([
        [ct,      -st,       0,       a],
        [st*ca,   ct*ca,    -sa,     -sa*d],
        [st*sa,   ct*sa,     ca,      ca*d],
        [0,         0,        0,        1]
    ])

# ============================================================
# 2. SYMBOLIC JOINT VARIABLES
# ============================================================
q = sym.symbols("q1:7")

# ============================================================
# 3. D1 MODIFIED DH PARAMETERS
# ============================================================
# Columns:
# [alpha, a, d, theta_offset]

DH_parameters = [
    [0,            0,       0.1096,  sym.pi  ],
    [sym.pi/2,     0,      -0.0280,  sym.pi/2],
    [0,            0.2693,  0.0266, -sym.pi  ],
    [sym.pi/2,    -0.042,   0.1979, -sym.pi  ],
    [sym.pi/2,     0.0001,  0.0001,  0       ],
    [sym.pi/2,    -0.0011, -0.0825,  sym.pi  ],
]


# ============================================================
# 4. CREATE INDIVIDUAL DH MATRICES
# ============================================================
A = []

for i, (alpha, a, d, theta_offset) in enumerate(DH_parameters):
    
    theta = q[i] + theta_offset
    
    A_i = DH_matrix(
        alpha,
        a,
        d,
        theta
    )
    
    A.append(A_i)

# ============================================================
# 5. FORWARD KINEMATICS
# ============================================================
T = [sym.eye(4)]      # T[0] = T_00

for A_i in A:
    T.append(T[-1] @ A_i)

# T[1] = T_01
# T[2] = T_02
# ...
# T[6] = T_06

# ============================================================
# 6. SET JOINT ANGLES
# ============================================================
zero_configuration = {
    q[i]: 0
    for i in range(6)
}

# ============================================================
# 7. EVALUATE TRANSFORMS
# ============================================================
T_numeric = [
    np.array(T_i.subs(zero_configuration)).astype(float)
    for T_i in T
]

# ============================================================
# 8. PRINT TRANSFORMATION MATRICES
# ============================================================
for i, T_i in enumerate(T_numeric):
    print(f"\nT_0{i}:")
    print(T_i)

# ============================================================
# 9. PLOT ROBOT FRAMES
# ============================================================
def plot_frames(transforms):
    """
    Plot all D1 coordinate frames.
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for i, T_i in enumerate(transforms):
        pt.plot_transform(
            ax=ax,
            A2B=T_i,
            s=0.1,
            name=f"Frame {i}"
        )

    # Dimensions are in metres
    ax.set_xlim(-0.5, 0.8)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.8)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.set_title("Unitree D1 Modified DH Frames")

    plt.show()


plot_frames(T_numeric)