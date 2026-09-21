#!/usr/bin/python3
# ============================================================
# ELEC 311 -- Lecture 8: Run This First
#
# The obvious starting guess is the arm straight out: q = (0, 0).
# Run it. Read the error message. Then we will talk about why.
# ============================================================

import numpy as np

l1, l2 = 0.20, 0.15

def jacobian(q1, q2):
    return np.array([
        [-l1*np.sin(q1) - l2*np.sin(q1 + q2), -l2*np.sin(q1 + q2)],
        [ l1*np.cos(q1) + l2*np.cos(q1 + q2),  l2*np.cos(q1 + q2)]])

q1, q2 = 0.0, 0.0
J = jacobian(q1, q2)

print("J =\n", J)
print("\ndet J =", np.linalg.det(J))
print("\nNow invert it:\n")
print(np.linalg.inv(J))
