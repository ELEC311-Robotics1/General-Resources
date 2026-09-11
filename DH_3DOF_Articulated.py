#%%
import sympy as sp
sp.init_printing(use_unicode=True) # Enables beautiful mathematical drawing

def dh_transform(theta, d, a, alpha):
    A = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                    [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                    [0, sp.sin(alpha), sp.cos(alpha), d],
                    [0, 0, 0, 1]])
    return A

# Generalized variables
q1, q2, q3 = sp.symbols('q1 q2 q3') 
q = sp.Matrix([q1, q2, q3])  
dq1, dq2, dq3 = sp.symbols('dq1 dq2 dq3')  
dq = sp.Matrix([dq1, dq2, dq3])  

# Robot parameters 3DOF articulated robot
L1, L2, L3 = sp.symbols('L1 L2 L3')
a = [0, L2, L3]  
d = [L1, 0, 0]  
alpha = [sp.pi/2, 0, 0]  
theta = [q1, q2, q3]  

n = len(a)  # Number of links (n = 3)

# Transformation Matrices from i to i-1
A = []
for i in range(n):
    A.append(dh_transform(theta[i], d[i], a[i], alpha[i]))

# Transformation Matrices from i to 0
T = []
prod = sp.eye(4)
for i in range(n):
    prod = prod * A[i]
    T.append(sp.simplify(prod))

# End-effector position (Top 3 rows, 4th column of the final matrix T[-1])
# In Python, row/col index 0:3 means rows 0,1,2 and index 3 is the 4th column
re = T[-1][0:3, 3]

# End-effector Rotation matrix (Top-left 3x3 of the final matrix T[-1])
Rn = T[-1][0:3, 0:3]

# Store rotation matrices for all links in a list
R = []
for i in range(n):
    R.append(T[i][0:3, 0:3])

# Print results to verify
#print("End-effector Position (re):")
#sp.pprint(re)
#print("\nEnd-effector Rotation (Rn):")
#sp.pprint(Rn)

"""
# For printing the transformation matrices, you can use the following code:
print("\nTransformation Matrices (T):")
for i, Ti in enumerate(T):
    print(f"T[{i}]:")
    sp.pprint(Ti)
"""

# End Effector Linear Velocity Jacobian Matrix
Jv = re.jacobian(q)  # Computes the full 3x3 partial derivative matrix automatically
#print("\nEnd-effector Linear Velocity Jacobian (Jv):")
#sp.pprint(sp.simplify(Jv)) # Adding simplify makes the output look much cleaner

# End Effector Angular Velocity Jacobian Matrix
# dRn = (dRn/dq1)*dq1 + (dRn/dq2)*dq2 + (dRn/dq3)*dq3
dRn = sp.zeros(3, 3)
for i in range(n):
    dRn += sp.diff(Rn, q[i]) * dq[i]

# Compute the skew-symmetric angular velocity matrix: [w] = dRn * Rn^T
Mat = sp.simplify(dRn * Rn.T)

# Extract the components [wx; wy; wz] from the skew-symmetric matrix
Aux = sp.Matrix([Mat[2, 1], Mat[0, 2], Mat[1, 0]])

# Extract the Angular Jacobian by taking the Jacobian with respect to dq
Jw = Aux.jacobian(dq)

#print("Angular Velocity Jacobian (Jw):")
#sp.pprint(sp.simplify(Jw))


# Joint types: 1 for Revolute, 0 for Prismatic. Matches your rho = [1, 1, 0]
# Note: Since your 3DOF articulated robot has 3 revolute joints, you might want [1, 1, 1]
rho = [1, 1, 1]  

# Create a 3D structure using a list of matrices
Jw_all = [sp.zeros(3, n) for _ in range(n)]

# Loop through each link frame i
for i in range(n):
    # Joint 1 (Index 0) always rotates around the base z-axis [0, 0, 1]
    Jw_all[i][0:3, 0] = rho[0] * sp.Matrix([0, 0, 1])
    
    # Fill in the contributions of subsequent joints up to the current link frame i
    for j in range(1, i + 1):
        # R[j-1] is the rotation matrix of frame j. 
        # Its 3rd column (index 2) is the z-axis of joint j+1.
        z_axis = R[j-1][0:3, 2]
        Jw_all[i][0:3, j] = rho[j] * z_axis

# Print the Angular Jacobian for the final link (the end-effector)
# print("Angular Velocity Jacobian for the final link (Jw):")
# sp.pprint(sp.simplify(Jw_all[-1]))

Jw

# %%
