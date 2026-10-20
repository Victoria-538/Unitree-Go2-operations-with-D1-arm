import DH_parameters as dh
import numpy as np

#Link masses
m_link = np.array([
    0.077233,  # base_link
    0.131740,  # Link1
    0.202130,  # Link2
    0.062900,  # Link3
    0.083332,  # Link4
    0.053817,  # Link5
    0.077892   # Link6
])

#Link CoM in their frames
base_com = np.array([
    -0.00048697,
    -2.9605e-12,
     0.017142,
    1
])
com_local = np.array([
    [0.0024649,    0.00010517,  0.032696,  1],  # Link1
    [0.0002018,    0.19201,    -0.027007,  1],  # Link2
    [0.015164,     0.044482,   -0.027461,  1],  # Link3
    [-0.00029556, -0.00016104,  0.091339,  1],  # Link4
    [0.040573,     0.0062891,  -0.023838,  1],  # Link5
    [-0.0068528,  -0.0000039973, 0.039705, 1],  # Link6
])


# Link CoM in base frame
com_base = []

for i in range(6):
    com = dh.T_numeric[i+1] @ com_local[i]
    com_base.append(com[:3])

    #print(f"\nLink {i+1} CoM in base frame:")
    #print(com[:3])

#arm CoM in base frame 
base_com_xyz = base_com[:3]

total_mass = np.sum(m_link)

arm_com = m_link[0] * base_com_xyz

for i in range(6):
    arm_com += m_link[i+1] * com_base[i]

arm_com /= total_mass

print("\nD1 arm CoM:")
print(arm_com)