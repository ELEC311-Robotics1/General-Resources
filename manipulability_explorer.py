#!/usr/bin/python3
# ============================================================
# ELEC 311 -- W5 Hands-On: MANIPULABILITY EXPLORER
#
# Jog the JOINTS. Watch the ellipse.
#
# The ellipse is every direction the hand can move, for one unit
# of joint effort. Long axis: the direction the hand moves most.
# Short axis: the direction it moves least. When the ellipse goes
# flat, there is a direction the hand cannot move AT ALL, at any
# joint speed. That is a singularity.
#
#   W / S     joint 1   (shoulder)      hold the key
#   O / K     joint 2   (elbow)
#   M         mark this pose on the workspace map
#   T         trail on / off  (where the hand has been)
#   C         clear the marks and the trail
#   P         save a screenshot  (shot_01.png, shot_02.png, ...)
#   R         reset          ESC / Q   quit
#
# Two joints, two sides of the keyboard: one person per joint, or
# one person with both hands. Your call.
#
# DELIVERABLE: six annotated screenshots, committed.
# ============================================================

import numpy as np
import pygame

# ============================================================
# STUDENT CONFIGURATION -- this is the part you change
# ============================================================

L1 = 0.20          # upper link length, metres
L2 = 0.15          # forearm length, metres

# Soft joint limits, DEGREES. The arm cannot leave this box.
# Start wide open. Then close them and see what you bought,
# and what it cost you.
Q1_MIN, Q1_MAX = -180.0, 180.0
Q2_MIN, Q2_MAX = -180.0, 180.0

JOG_SPEED = 60.0   # degrees per second while a key is held

# ============================================================
# DO NOT TOUCH ANYTHING BELOW THIS LINE
# ============================================================
# Everything from here down is the instrument: the Jacobian, the
# ellipse construction, and the drawing. If you change it, your
# measurements stop meaning anything and your screenshots stop
# being evidence. Edit the block above instead.
# ============================================================

W, H = 1080, 640
PANEL = 620
PPM = 760                  # pixels per metre, arm view
PPM_MAP = 430              # pixels per metre, workspace map
ELLIPSE_PPM = 270          # pixels per (metre/second)
FLAT = 0.02                # w below this counts as "flat"
TRAIL_MAX = 4000           # hand positions kept in the trail

BG = (8, 8, 15)
CARD = (18, 19, 31)
CARD_HI = (78, 84, 122)    # trail dots on the map
FG = (216, 220, 232)
DIM = (110, 117, 144)
CYAN = (34, 211, 238)
MAGENTA = (220, 20, 150)
AMBER = (255, 176, 32)
LIME = (158, 232, 75)

ARM_O = (300, 330)                 # base of the arm, screen px
MAP_O = (PANEL + 225, 300)         # base on the workspace map


def fk(q1, q2):
    """Hand position, metres."""
    return np.array([L1 * np.cos(q1) + L2 * np.cos(q1 + q2),
                     L1 * np.sin(q1) + L2 * np.sin(q1 + q2)])


def elbow(q1):
    return np.array([L1 * np.cos(q1), L1 * np.sin(q1)])


def jacobian(q1, q2):
    """Hand velocity per unit joint velocity. Two columns, one per joint."""
    return np.array([
        [-L1 * np.sin(q1) - L2 * np.sin(q1 + q2), -L2 * np.sin(q1 + q2)],
        [ L1 * np.cos(q1) + L2 * np.cos(q1 + q2),  L2 * np.cos(q1 + q2)]])


def ellipse_points(J, n=160):
    """
    Push a ring of unit joint-velocity vectors through J.

    The circle of inputs comes out as an ellipse of outputs. No
    decomposition, no eigenvectors: just the image of a set.
    """
    th = np.linspace(0, 2 * np.pi, n)
    unit = np.vstack([np.cos(th), np.sin(th)])   # every unit dq
    return J @ unit                              # the hand velocities


def wrap(a):
    """Fold an angle into (-pi, pi]. A revolute joint has no end."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def to_px(p, origin, ppm):
    """Metres (x right, y up) -> pixels (x right, y down)."""
    return (int(origin[0] + p[0] * ppm), int(origin[1] - p[1] * ppm))


def main():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("ELEC 311 -- Manipulability Explorer")
    clock = pygame.time.Clock()
    f_big = pygame.font.Font(None, 30)
    f = pygame.font.Font(None, 24)
    f_sm = pygame.font.Font(None, 20)

    q1, q2 = np.radians(30.0), np.radians(60.0)
    marks = []
    trail = []
    show_trail = True
    shot = 0
    running = True

    while running:
        dt = clock.tick(60) / 1000.0

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif ev.key == pygame.K_r:
                    q1, q2 = np.radians(30.0), np.radians(60.0)
                elif ev.key == pygame.K_m:
                    marks.append(fk(q1, q2))
                elif ev.key == pygame.K_c:
                    marks.clear()
                    trail.clear()
                elif ev.key == pygame.K_t:
                    show_trail = not show_trail
                elif ev.key == pygame.K_p:
                    shot += 1
                    name = f"shot_{shot:02d}.png"
                    pygame.image.save(screen, name)
                    print(f"  saved {name}   "
                          f"q1={np.degrees(q1):7.2f}  q2={np.degrees(q2):7.2f}  "
                          f"w={abs(np.linalg.det(jacobian(q1, q2))):.5f}")

        keys = pygame.key.get_pressed()
        step = np.radians(JOG_SPEED) * dt
        if keys[pygame.K_w]:
            q1 += step
        if keys[pygame.K_s]:
            q1 -= step
        if keys[pygame.K_o]:
            q2 += step
        if keys[pygame.K_k]:
            q2 -= step

        # A revolute joint keeps turning: wrap first, so the arm rolls
        # through +/-180 instead of hitting a wall that is not there.
        q1, q2 = wrap(q1), wrap(q2)

        # Then apply the soft limits from the student block -- but only
        # when they actually restrict something. A full 360 span is not
        # a limit, and clamping to it is what built the false wall.
        if Q1_MAX - Q1_MIN < 360.0:
            q1 = np.clip(q1, np.radians(Q1_MIN), np.radians(Q1_MAX))
        if Q2_MAX - Q2_MIN < 360.0:
            q2 = np.clip(q2, np.radians(Q2_MIN), np.radians(Q2_MAX))

        J = jacobian(q1, q2)
        w = abs(np.linalg.det(J))
        hand = fk(q1, q2)

        # breadcrumb trail: every distinct hand position, capped
        if not trail or np.linalg.norm(hand - trail[-1]) > 2e-3:
            trail.append(hand)
            if len(trail) > TRAIL_MAX:
                del trail[0]
        elb = elbow(q1)
        flat = w < FLAT

        # ---------------- draw ----------------
        screen.fill(BG)
        pygame.draw.line(screen, CARD, (PANEL, 0), (PANEL, H), 2)

        # --- arm panel: workspace rings for context
        pygame.draw.circle(screen, CARD, ARM_O, int((L1 + L2) * PPM), 1)
        if abs(L1 - L2) > 1e-6:
            pygame.draw.circle(screen, CARD, ARM_O, int(abs(L1 - L2) * PPM), 1)

        # --- the ellipse, at the hand
        pts = ellipse_points(J)
        poly = [(int(ARM_O[0] + hand[0] * PPM + pts[0, i] * ELLIPSE_PPM),
                 int(ARM_O[1] - hand[1] * PPM - pts[1, i] * ELLIPSE_PPM))
                for i in range(pts.shape[1])]
        pygame.draw.polygon(screen, MAGENTA if flat else CYAN, poly, 2)

        # --- the arm on top
        p0 = to_px(np.array([0.0, 0.0]), ARM_O, PPM)
        p1 = to_px(elb, ARM_O, PPM)
        p2 = to_px(hand, ARM_O, PPM)
        pygame.draw.line(screen, FG, p0, p1, 6)
        pygame.draw.line(screen, FG, p1, p2, 6)
        for p in (p0, p1):
            pygame.draw.circle(screen, AMBER, p, 7)
        pygame.draw.circle(screen, LIME, p2, 6)

        # --- readouts
        y = 18
        screen.blit(f_big.render("MANIPULABILITY EXPLORER", True, CYAN), (18, y))
        y += 34
        for label, val in (("joint 1", f"{np.degrees(q1):8.2f} deg"),
                           ("joint 2", f"{np.degrees(q2):8.2f} deg"),
                           ("hand x ", f"{hand[0]:8.3f} m"),
                           ("hand y ", f"{hand[1]:8.3f} m")):
            screen.blit(f.render(f"{label}  {val}", True, FG), (18, y))
            y += 24

        y += 8
        screen.blit(f.render(f"w = |det J| = {w:.5f}", True,
                             MAGENTA if flat else LIME), (18, y))
        y += 24
        screen.blit(f_sm.render(f"ellipse area = pi*w = {np.pi * w:.5f}",
                                True, DIM), (18, y))
        y += 22
        if flat:
            screen.blit(f.render("FLAT -- a direction is lost", True,
                                 MAGENTA), (18, y))

        # --- workspace map panel
        mx, my = MAP_O
        screen.blit(f_big.render("WORKSPACE", True, CYAN), (PANEL + 24, 18))
        screen.blit(f_sm.render("press M to mark this pose", True, DIM),
                    (PANEL + 24, 48))
        pygame.draw.circle(screen, AMBER, (mx, my), int((L1 + L2) * PPM_MAP), 1)
        if abs(L1 - L2) > 1e-6:
            pygame.draw.circle(screen, AMBER, (mx, my),
                               int(abs(L1 - L2) * PPM_MAP), 1)
        pygame.draw.circle(screen, DIM, (mx, my), 4)
        if show_trail:
            for t in trail:
                screen.set_at(to_px(t, MAP_O, PPM_MAP), CARD_HI)
        for m in marks:
            pygame.draw.circle(screen, MAGENTA, to_px(m, MAP_O, PPM_MAP), 5)
        pygame.draw.circle(screen, LIME, to_px(hand, MAP_O, PPM_MAP), 6, 2)

        screen.blit(f_sm.render(f"marks: {len(marks)}   trail: "
                                f"{'on' if show_trail else 'off'}   (C clears)",
                                True, DIM),
                    (PANEL + 24, H - 90))
        screen.blit(f_sm.render("W/S joint 1    O/K joint 2", True, DIM),
                    (PANEL + 24, H - 66))
        screen.blit(f_sm.render("M mark   T trail   P shot   R reset   Q quit",
                                True, DIM), (PANEL + 24, H - 44))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()