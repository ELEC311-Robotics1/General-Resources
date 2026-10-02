#!/usr/bin/python3
# ============================================================
# ELEC 311 -- Kinematic Control of a 2-DoF Planar Arm
#
# Model: dot_X = J dot_q
# Task X -> X_des
# X_e = X - X_des
# dot_q = J^+ u
# u = dot_X_des - alpha X_e
#
# ------------------------------------------------------------
# ROBOTICS, CYBORGS & ASSOCIATES -- CONTROLS GROUP
#
# Spec RCA-SL-120:
#   No joint shall exceed 120 deg/s at any point of any
#   commissioned motion. This is not a tuning preference. It is
#   what keeps the gearbox, the horn, and the operator intact.
#
# There is no formula that gives you max ||dot_q|| ahead of time.
# It depends on the path, the speed, the pose, and sigma, all at
# once. You will not solve for it. You will RUN it, MEASURE it,
# and CHANGE something. That is the job.
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================
# CONFIGURATION -- the only lines you are asked to change
# ============================================================
SIGMA = 0.0            # damping. 0.0 = pure pseudo-inverse
JOINT_SPEED_LIMIT = 120.0   # deg/s, spec RCA-SL-120
TRANSIENT_SKIP = 6.0        # s. Start-up is not commissioning data.

# Time parameters
tf = 20.0
dt = 0.01
time = np.arange(0, tf, dt)
steps = len(time)

# Kinematic parameters
l1, l2 = 0.20, 0.15
REACH = l1 + l2                 # outer workspace boundary
INNER = abs(l1 - l2)            # inner workspace boundary (the hole)

# Control parameters
alpha = 1.0


# Forward kinematics
def fk(q1, q2):
    return np.array([l1*np.cos(q1) + l2*np.cos(q1 + q2),
                     l1*np.sin(q1) + l2*np.sin(q1 + q2)])


# Linear Velocity Jacobian
def jacobian(q1, q2):
    return np.array([
        [-l1*np.sin(q1) - l2*np.sin(q1 + q2), -l2*np.sin(q1 + q2)],
        [ l1*np.cos(q1) + l2*np.cos(q1 + q2),  l2*np.cos(q1 + q2)]])


# Pseudo-inverse of Jacobian (using stable solve operation)
def Pseudo_Inv(J, s=1e-3):
    m, n = J.shape
    s2_I = (s ** 2) * np.eye(min(m, n))
    if m < n:  # under-determined
        return J.T @ np.linalg.solve(J @ J.T + s2_I, np.eye(m))
    else:  # over-determined / square
        return np.linalg.solve(J.T @ J + s2_I, J.T)


# Initial joint conditions (positioned close to the trajectory start)
q1, q2 = 0.5, 0.8

# ============================================================
# TRAJECTORY -- replace this block, and only this block
#
# It must produce two arrays of shape (2, steps):
#   X_des[:, i]      where the hand should be at time[i]
#   dot_X_des[:, i]  how fast it should be moving there
#
# dot_X_des is the analytic derivative of X_des. Differentiate
# by hand. A wrong derivative still converges -- it just lags,
# and it will not announce itself.
# ============================================================
cx, cy = 0.2, 0.15              # centre of the circle
times = 3                       # number of complete revolutions
omega = times * 2 * np.pi / tf  # angular frequency

r_max_reach = REACH - np.sqrt(cx**2 + cy**2)   # = 0.10 exactly

radius = 0.5 * r_max_reach      # (a) comfortable
#radius = 1.0 * r_max_reach    # (b) exactly touching the boundary
#radius = r_max_reach + 0.02   # (c) DELIBERATELY 2 cm BEYOND REACH

X_des = np.array([cx + radius * np.cos(omega * time),
                  cy + radius * np.sin(omega * time)])
dot_X_des = np.array([-radius * omega * np.sin(omega * time),
                       radius * omega * np.cos(omega * time)])
# ============================ end of trajectory block =======

# Log arrays for animation and for the report
Q1 = []
Q2 = []
DQ = []          # joint speeds, deg/s
XE = []          # task-space error magnitude, m

# Simulation loop
for i in range(steps):
    # Current task space position
    X = fk(q1, q2)

    # Compute error
    X_e = X - X_des[:, i]

    # Compute control input (feedforward + feedback tracking)
    u = dot_X_des[:, i] - alpha * X_e

    # Compute joint velocities using regularized pseudo-inverse
    Jm = jacobian(q1, q2)
    dq = Pseudo_Inv(Jm, SIGMA) @ u

    # Update joint angles (Euler integration)
    q1 += dq[0] * dt
    q2 += dq[1] * dt

    # Log the configurations
    Q1.append(q1)
    Q2.append(q2)
    DQ.append(np.degrees(np.abs(dq)))
    XE.append(np.linalg.norm(fk(q1, q2) - X_des[:, i]))


# ============================================================
# COMMISSIONING REPORT
#
# Everything below is measurement. Nothing here was solved for.
# ============================================================
def report():
    DQa = np.array(DQ)
    XEa = np.array(XE)
    k = int(TRANSIENT_SKIP / dt)          # ignore the start-up transient
    r = np.linalg.norm(X_des, axis=0)     # target distance from the base
    outside = np.count_nonzero((r > REACH) | (r < INNER))

    rows = [
        ("sigma",                f"{SIGMA:.4f}",                  "",        ""),
        ("alpha",                f"{alpha:.2f} 1/s",              "",        ""),
        ("path radius",          f"{radius*1000:.1f} mm",         "",        ""),
        ("target r, min",        f"{r.min()*1000:.1f} mm",        f"> {INNER*1000:.0f}", 
                                 "ok" if r.min() >= INNER else "OUTSIDE"),
        ("target r, max",        f"{r.max()*1000:.1f} mm",        f"< {REACH*1000:.0f}",
                                 "ok" if r.max() <= REACH else "OUTSIDE"),
        ("samples off-workspace", f"{outside} / {steps}",         "0",
                                 "ok" if outside == 0 else f"{100*outside/steps:.1f}%"),
        ("max speed, joint 1",   f"{DQa[:, 0].max():.1f} deg/s",  "",        ""),
        ("max speed, joint 2",   f"{DQa[:, 1].max():.1f} deg/s",  "",        ""),
        ("MAX JOINT SPEED",      f"{DQa.max():.1f} deg/s",
                                 f"< {JOINT_SPEED_LIMIT:.0f}",
                                 "PASS" if DQa.max() <= JOINT_SPEED_LIMIT else "FAIL"),
        ("mean error",           f"{XEa[k:].mean()*1000:.2f} mm", "",        ""),
        ("max error",            f"{XEa[k:].max()*1000:.2f} mm",  "",        ""),
    ]

    w = 78
    print("\n" + "=" * w)
    print("  ROBOTICS, CYBORGS & ASSOCIATES -- COMMISSIONING REPORT")
    print("=" * w)
    print(f"  {'quantity':<24}{'measured':>16}{'spec':>14}{'verdict':>16}")
    print("  " + "-" * (w - 4))
    for name, val, spec, verdict in rows:
        print(f"  {name:<24}{val:>16}{spec:>14}{verdict:>16}")
    print("=" * w)
    print(f"  errors measured after t = {TRANSIENT_SKIP:.0f} s (start-up excluded)")
    if DQa.max() > JOINT_SPEED_LIMIT:
        print("  DO NOT SHIP. A joint exceeds RCA-SL-120.")
    print("=" * w + "\n")


report()

# Animation setup
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, aspect='equal', xlim=(-0.1, 0.4), ylim=(-0.1, 0.4))
line, = ax.plot([], [], 'o-', lw=3, color='blue', label='Robot Arm')
target_path, = ax.plot(X_des[0], X_des[1], 'y--', lw=1, label='Target Path')
ax.legend()


def Update(frame):
    tht1 = Q1[frame]
    tht2 = Q2[frame]
    x0, y0 = 0, 0
    x1, y1 = l1 * np.cos(tht1), l1 * np.sin(tht1)
    x2, y2 = x1 + l2 * np.cos(tht1 + tht2), y1 + l2 * np.sin(tht1 + tht2)
    line.set_data([x0, x1, x2], [y0, y1, y2])
    return line,


# Create and run the animation
ani = FuncAnimation(fig, Update, frames=steps, interval=dt*100,
                    blit=True, repeat=False)
plt.title("Two-DOF Planar Arm Tracking a Circular Trajectory")
plt.grid(True)
plt.show()