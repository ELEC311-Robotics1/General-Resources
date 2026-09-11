import sympy as sp

def dh_transform(theta, d, a, alpha):
    A = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                    [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                    [0, sp.sin(alpha), sp.cos(alpha), d],
                    [0, 0, 0, 1]])
    return A

# Generalized variables
q1, q2 = sp.symbols('q1 q2') 
q = sp.Matrix([q1, q2])  
dq1, dq2 = sp.symbols('dq1 dq2')  
dq = sp.Matrix([dq1, dq2])  

# Robot parameters 
L1, L2 = sp.symbols('L1 L2')
a = [L1, L2]  
d = [0, 0]  
alpha = [0, 0]  
theta = [q1, q2]  

n = len(a)  # Number of links (n = 2)

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
print("End-effector Position (re):")
sp.pprint(re)
print("\nEnd-effector Rotation (Rn):")
sp.pprint(Rn)
