#!/usr/bin/python3
# ============================================================
# ELEC 311 -- Lecture 8: Jog the HAND
#
# In Week 2 you drove the JOINTS and watched the hand.
# Now you drive the HAND and the solver finds the joints.
#
# The kinematics never changed. Only what feeds it.
#
#   w / s   move target up / down      (1 cm)
#   a / d   move target left / right   (1 cm)
#   r       reset
#   q       quit
# ============================================================

import numpy as np

l1, l2 = 0.20, 0.15

def fk(q1, q2):
    return np.array([l1*np.cos(q1) + l2*np.cos(q1 + q2),
                     l1*np.sin(q1) + l2*np.sin(q1 + q2)])

def jacobian(q1, q2):
    return np.array([
        [-l1*np.sin(q1) - l2*np.sin(q1 + q2), -l2*np.sin(q1 + q2)],
        [ l1*np.cos(q1) + l2*np.cos(q1 + q2),  l2*np.cos(q1 + q2)]])

def solve(target, q1, q2):
    """Newton-Raphson. Starts from where the arm already is."""
    for i in range(50):
        e = fk(q1, q2) - target
        if np.linalg.norm(e) < 1e-9:
            return q1, q2, i, True
        Jm = jacobian(q1, q2)
        if abs(np.linalg.det(Jm)) < 1e-9:
            return q1, q2, i, False        # singular: cannot step
        dq = np.linalg.inv(Jm) @ e
        q1, q2 = q1 - dq[0], q2 - dq[1]
    return q1, q2, 50, False

HELP = """
  w / s   target up / down      a / d   target left / right
  r       reset                 q       quit
"""

START = np.array([0.25, 0.15])
q1, q2 = 0.3, 0.5
target = START.copy()
q1, q2, _, _ = solve(target, q1, q2)

print(HELP)
while True:
    r = np.linalg.norm(target)
    print(f"  target ({target[0]:6.3f}, {target[1]:6.3f})  r={r:.3f}   "
          f"q1={np.degrees(q1):7.2f}  q2={np.degrees(q2):7.2f}")
    c = input("  > ").strip().lower()
    if c == "q":
        break
    if c == "r":
        target = START.copy()
        q1, q2 = 0.3, 0.5
        q1, q2, _, _ = solve(target, q1, q2)
        continue
    step = {"w": (0, .01), "s": (0, -.01), "a": (-.01, 0), "d": (.01, 0)}
    if c not in step:
        continue
    new = target + np.array(step[c])
    nq1, nq2, iters, ok = solve(new, q1, q2)
    if ok:
        target, q1, q2 = new, nq1, nq2
        print(f"    solved in {iters} iterations")
    else:
        print(f"    NO SOLUTION at ({new[0]:.3f}, {new[1]:.3f}) "
              f"-- r={np.linalg.norm(new):.3f}, reach is "
              f"{abs(l1-l2):.2f} to {l1+l2:.2f}")
