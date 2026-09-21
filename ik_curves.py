#!/usr/bin/python3
# ============================================================
# ELEC 311 -- Lecture 8: Draw a Shape
#
# One target was a warm-up. A SHAPE is a list of targets.
# Solve each one, keep the answer, feed it to the next.
#
#   python ik_curves.py circle
#   python ik_curves.py eight
#   python ik_curves.py spiral
# ============================================================

import sys
import numpy as np
import matplotlib.pyplot as plt

l1, l2 = 0.20, 0.15

def fk(q1, q2):
    return np.array([l1*np.cos(q1) + l2*np.cos(q1 + q2),
                     l1*np.sin(q1) + l2*np.sin(q1 + q2)])

def jacobian(q1, q2):
    return np.array([
        [-l1*np.sin(q1) - l2*np.sin(q1 + q2), -l2*np.sin(q1 + q2)],
        [ l1*np.cos(q1) + l2*np.cos(q1 + q2),  l2*np.cos(q1 + q2)]])

def solve(target, q1, q2):
    for _ in range(50):
        e = fk(q1, q2) - target
        if np.linalg.norm(e) < 1e-10:
            return q1, q2, True
        dq = np.linalg.inv(jacobian(q1, q2)) @ e
        q1, q2 = q1 - dq[0], q2 - dq[1]
    return q1, q2, False

# ---------- the shapes, as parametric equations ----------
def curve(name, n=200):
    t = np.linspace(0, 2*np.pi, n)
    if name == "circle":
        return 0.22 + 0.05*np.cos(t), 0.08 + 0.05*np.sin(t)
    if name == "eight":
        return 0.22 + 0.07*np.cos(t), 0.09 + 0.05*np.sin(t)*np.cos(t)
    if name == "spiral":
        r = 0.02 + 0.035*t/(2*np.pi)
        return 0.22 + r*np.cos(3*t), 0.10 + r*np.sin(3*t)
    raise SystemExit("choose: circle | eight | spiral")

name = sys.argv[1] if len(sys.argv) > 1 else "circle"
xs, ys = curve(name)

q1, q2 = 0.3, 0.5
hx, hy, Q1, Q2 = [], [], [], []
for x, y in zip(xs, ys):
    q1, q2, ok = solve(np.array([x, y]), q1, q2)
    if not ok:
        print(f"  no solution at ({x:.3f}, {y:.3f})")
        continue
    p = fk(q1, q2)
    hx.append(p[0]); hy.append(p[1]); Q1.append(q1); Q2.append(q2)

print(f"  {name}: solved {len(hx)} of {len(xs)} points")
print(f"  max error: {max(np.hypot(np.array(hx)-xs, np.array(hy)-ys)):.2e} m")

# ---------- plot ----------
plt.rcParams.update({"axes.facecolor": (8/255, 8/255, 15/255),
                     "figure.facecolor": (8/255, 8/255, 15/255),
                     "text.color": "white", "axes.labelcolor": "white",
                     "xtick.color": "white", "ytick.color": "white",
                     "axes.edgecolor": (0.3, 0.3, 0.3)})
fig, ax = plt.subplots(1, 2, figsize=(11, 5))

ax[0].plot(xs, ys, color=(220/255, 20/255, 150/255), lw=3, label="target")
ax[0].plot(hx, hy, color=(34/255, 211/255, 238/255), lw=1, ls="--", label="hand")
ax[0].set_aspect("equal", "box"); ax[0].grid(True, color=(0.2, 0.2, 0.2))
ax[0].set_title(f"task space -- {name}"); ax[0].legend()
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")

ax[1].plot(np.degrees(Q1), color=(34/255, 211/255, 238/255), label="q1")
ax[1].plot(np.degrees(Q2), color=(255/255, 176/255, 32/255), label="q2")
ax[1].grid(True, color=(0.2, 0.2, 0.2)); ax[1].legend()
ax[1].set_title("joint space"); ax[1].set_xlabel("point"); ax[1].set_ylabel("deg")

plt.tight_layout(); plt.show()
