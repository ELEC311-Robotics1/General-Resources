import numpy as np

def ik_2dof(x, y, l1=0.20, l2=0.15):

    # Law of cosines
    c2 = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)
    # Check reachability
    if c2 < -1.0 or c2 > 1.0:
        raise ValueError("Point is outside the reachable workspace.")
    c2 = np.clip(c2, -1.0, 1.0); s2 = np.sqrt(1 - c2**2) 
    q2 = np.arctan2(s2, c2) # Positive elbow down solution, negative for elbow up
    k1 = l1 + l2*np.cos(q2); k2 = l2*np.sin(q2)
    q1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return q1, q2

l1 = 0.05; l2 = 0.20; l3 = 0.15
x = 0.3; y = 0.1; z = 0.15

q1 = np.arctan2(y, x)
r = np.sqrt(x**2 + y**2)
q2, q3 = ik_2dof(r, z-l1, l2, l3)

print(f"Joint angles for position ({x}, {y}, {z}):")
print(f"q1: {np.degrees(q1):.2f} degrees")
print(f"q2: {np.degrees(q2):.2f} degrees")
print(f"q3: {np.degrees(q3):.2f} degrees")
