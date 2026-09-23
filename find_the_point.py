#!/usr/bin/python3
# ============================================================
# ELEC 311 -- W4 Hands-On [E]: Find the Point
#
# You name a point. The arm goes there.
#
#     solve_ik()      point in space  ->  DH angles
#     dh_to_code()    DH angles       ->  servo angles
#     safe_to_send()  servo angles    ->  yes or no
#     set_all_deg()   servo angles    ->  the arm moves
#
# TWO BLANKS, both marked. Everything else is written.
#
# ------------------------- SAFETY -------------------------
# Run ARM_CHECK_Station.py first. Nobody works alone.
# Nothing is sent to the arm until safe_to_send() returns True.
# ----------------------------------------------------------
#
#   python find_the_point.py          solve only, nothing moves
#   python find_the_point.py --run    solve, then move the arm
# ============================================================

import sys

import numpy as np
import matplotlib.pyplot as plt

from openarm import D1, L1, L2, L3, ALPHA, BETA, JOINT_LIMITS_DEG

# ---- DH table. theta_i = q_i, no offsets. Lengths in mm. ----
a     = [0.0,     L1,  L2,  L3]
alpha = [np.pi/2, 0.0, 0.0, 0.0]
d     = [D1,      0.0, 0.0, 0.0]


def H(theta, d, a, alpha):
    """One homogeneous transform. Same as Lecture 5."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0.,     sa,     ca,    d],
                     [0.,     0.,     0.,   1.]])


def fk_dh(theta):
    """DH angles (rad) -> hand position (mm)."""
    T = np.eye(4)
    for i in range(4):
        T = T @ H(theta[i], d[i], a[i], alpha[i])
    return T[0:3, 3]


def jacobian_dh(theta, h=1e-6):
    """3 rows: a point is 3 numbers. 4 columns: the arm has 4 joints."""
    J = np.zeros((3, 4))
    for i in range(4):
        up, dn = np.array(theta, float), np.array(theta, float)
        up[i] += h
        dn[i] -= h
        J[:, i] = (fk_dh(up) - fk_dh(dn)) / (2*h)
    return J


# ---- Encoder zero is not DH zero. You verified this in W3. ----
def code_to_dh(q_code_deg):
    q = np.radians(q_code_deg)
    return np.array([q[0], -q[1] + ALPHA, -q[2] - ALPHA - BETA, -q[3]])


def dh_to_code(theta):
    q = np.degrees(np.array([theta[0],
                             ALPHA - theta[1],
                             -theta[2] - ALPHA - BETA,
                             -theta[3]]))
    return q #(q + 180.0) % 360.0 - 180.0      # fold into [-180, 180]


# ---- Newton-Raphson. Same loop as Lecture 8. ----
def solve_ik(target, guess, max_iters=100, tol=1e-3):
    theta = np.array(guess, float)
    history = []
    for i in range(max_iters):
        e = ____________________          # <<<< BLANK 1
        history.append(np.linalg.norm(e))
        if np.linalg.norm(e) < tol:
            return theta, history, True
        J = jacobian_dh(theta)
        dtheta = ____________________     # <<<< BLANK 2
        theta = theta - dtheta
    return theta, history, False


# ---- The arm has limits. The solver does not know that. ----
def safe_to_send(q_code_deg):
    ok = True
    for mid, val in zip([11, 12, 13, 14], q_code_deg):
        lo, hi = JOINT_LIMITS_DEG[mid]
        if not (lo <= val <= hi):
            print(f"      ID {mid}: {val:8.2f} deg OUTSIDE [{lo}, {hi}]")
            ok = False
    return ok


# ============================================================
TARGETS = {"T1": np.array([250.0,   0.0, 150.0]),
           "T2": np.array([200.0, 120.0, 180.0]),
           "T3": np.array([420.0,   0.0, 300.0])}

GUESS_CODE = [0.0, 30.0, -40.0, 10.0]


def main():
    guess = code_to_dh(GUESS_CODE)
    results = {}

    for name, target in TARGETS.items():
        theta, history, converged = solve_ik(target, guess)   # 1. IK
        q_code = dh_to_code(theta)                            # 2. mapping
        safe = converged and safe_to_send(q_code)             # 3. check

        print(f"\n  {name}  target {target} mm")
        print(f"      iterations {len(history)}   converged {converged}")
        print(f"      error      {history[-1]:.4f} mm")
        print(f"      servo      {np.round(q_code, 2)}")
        print(f"      FK check   {np.round(fk_dh(theta), 2)}")
        print(f"      SAFE TO SEND: {safe}")
        results[name] = (history, q_code, safe)

    plt.figure(figsize=(7, 5))
    for name, (history, _, _) in results.items():
        plt.semilogy(history, marker="o", markersize=3, label=name)
    plt.xlabel("iteration"); plt.ylabel("error norm (mm)")
    plt.title("Find the Point -- convergence")
    plt.grid(True, which="both"); plt.legend()
    plt.savefig("convergence.png", dpi=120)
    print("\n  Saved convergence.png")

    if "--run" not in sys.argv:
        print("  Solve-only. Re-run with --run to move the arm.\n")
        return

    from openarm import OpenArmX
    arm = OpenArmX()
    arm.home()
    arm.set_profile(vel=30, acc=15)
    try:
        for name, (_, q_code, safe) in results.items():
            if not safe:
                print(f"  {name}: SKIPPED")
                continue
            input(f"  {name} -> {np.round(q_code, 1)}   Enter to move ")
            arm.set_all_deg(list(q_code))                     # 4. write
            arm.wait_until_done()
            input("      measure the tip, then Enter ")
    finally:
        arm.close()
        print("=^..^=")


if __name__ == "__main__":
    main()
