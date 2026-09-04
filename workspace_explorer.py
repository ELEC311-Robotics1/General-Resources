#!/usr/bin/python3
# ============================================================
# ELEC 311 -- W2 Hands-On [S]: Workspace Explorer
#
# A two-link planar arm. You drive the JOINTS. The screen shows
# you where the HAND ends up.
#
# Nothing in this program is measured. Every dot on the screen
# came out of the forward kinematics you built in Lecture 4:
#
#     x = L1*cos(q1) + L2*cos(q1 + q2)
#     y = L1*sin(q1) + L2*sin(q1 + q2)
#
# CONTROLS
#   W / S    shoulder  q1   (counter-clockwise / clockwise)
#   O / K    elbow     q2
#   L        joint limits ON / OFF     <- do this one twice
#   F        freeze the elbow          <- and this one
#   C        clear the trail   (Task 4: leave the trail ON)
#   R        reset the arm to zero
#   ESC      quit
#
# DELIVERABLE
#   Paint the workspace with limits OFF, screenshot it.
#   Turn limits ON, paint again, screenshot that.
#   Annotate both: outer boundary, inner boundary, and every
#   edge the limits carved out. Commit to your team repo.
#
# TASK 4 -- uncomment SQUARE below and drive the hand around
#   the four corners. Then explain why the edges are hard.
# ============================================================

import math
import sys

import pygame

# ---------- the robot ----------------------------------------
L1 = 0.20          # metres, shoulder to elbow
L2 = 0.15          # metres, elbow to hand

Q1_LIMIT = (-90.0, 90.0)     # degrees
Q2_LIMIT = (-135.0, 135.0)

STEP = 1.2         # degrees per frame while a key is held

# ---- TASK 4 CHALLENGE ---------------------------------------
# Uncomment to show four target corners. Drive the hand around
# them: corners are easy, the straight edges are not.
# SQUARE = [(0.10, 0.11), (0.22, 0.11),
#           (0.22, 0.23), (0.10, 0.23)]
SQUARE = None

# ---------- the window ---------------------------------------
W, H = 1100, 700
ORIGIN = (330, 430)          # where the shoulder is drawn
SCALE = 700.0                # pixels per metre

BG = (8, 8, 15)
CARD = (18, 19, 31)
FG = (216, 220, 232)
DIM = (110, 117, 144)
CYAN = (34, 211, 238)
MAGENTA = (220, 20, 150)
LIME = (158, 232, 75)
AMBER = (255, 176, 32)


def forward_kinematics(q1, q2):
    """Joint angles (radians) -> hand position (metres). No sensors."""
    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y


def elbow_position(q1):
    return L1 * math.cos(q1), L1 * math.sin(q1)


def to_screen(x, y):
    """Metres -> pixels. Screen y grows downward, so flip it."""
    return int(ORIGIN[0] + x * SCALE), int(ORIGIN[1] - y * SCALE)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("ELEC 311 -- Workspace Explorer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas,dejavusansmono,monospace", 19)
    big = pygame.font.SysFont("consolas,dejavusansmono,monospace", 23, bold=True)

    q1_deg, q2_deg = 0.0, 0.0
    trail: list[tuple[int, int]] = []
    limits_on = False
    elbow_frozen = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                elif event.key == pygame.K_l:
                    limits_on = not limits_on
                elif event.key == pygame.K_f:
                    elbow_frozen = not elbow_frozen
                elif event.key == pygame.K_c:
                    trail.clear()
                elif event.key == pygame.K_r:
                    q1_deg, q2_deg = 0.0, 0.0
                    trail.clear()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            q1_deg += STEP
        if keys[pygame.K_s]:
            q1_deg -= STEP
        if not elbow_frozen:
            if keys[pygame.K_o]:
                q2_deg += STEP
            if keys[pygame.K_k]:
                q2_deg -= STEP

        if limits_on:
            q1_deg = clamp(q1_deg, *Q1_LIMIT)
            q2_deg = clamp(q2_deg, *Q2_LIMIT)

        q1, q2 = math.radians(q1_deg), math.radians(q2_deg)
        ex, ey = elbow_position(q1)
        hx, hy = forward_kinematics(q1, q2)

        point = to_screen(hx, hy)
        if not trail or point != trail[-1]:
            trail.append(point)

        # ---------- draw ----------
        screen.fill(BG)

        for px, py in trail:
            screen.set_at((px, py), LIME)
            screen.set_at((px + 1, py), LIME)
            screen.set_at((px, py + 1), LIME)

        if SQUARE:
            pts = [to_screen(*c) for c in SQUARE]
            pygame.draw.lines(screen, MAGENTA, True, pts, 2)
            for p in pts:
                pygame.draw.circle(screen, MAGENTA, p, 6)

        pygame.draw.line(screen, DIM, (ORIGIN[0] - 300, ORIGIN[1]),
                         (ORIGIN[0] + 300, ORIGIN[1]), 1)
        pygame.draw.line(screen, DIM, (ORIGIN[0], ORIGIN[1] - 300),
                         (ORIGIN[0], ORIGIN[1] + 250), 1)

        pygame.draw.line(screen, CYAN, ORIGIN, to_screen(ex, ey), 6)
        pygame.draw.line(screen, MAGENTA if elbow_frozen else CYAN,
                         to_screen(ex, ey), point, 6)
        pygame.draw.circle(screen, FG, ORIGIN, 9)
        pygame.draw.circle(screen, FG, to_screen(ex, ey), 7)
        pygame.draw.circle(screen, AMBER, point, 7)

        # ---------- readout ----------
        panel = pygame.Rect(790, 30, 285, 250)
        pygame.draw.rect(screen, CARD, panel, border_radius=8)

        lines = [
            (big, "JOINTS  (you drive)", CYAN),
            (font, f"  q1 = {q1_deg:8.2f} deg", FG),
            (font, f"  q2 = {q2_deg:8.2f} deg", MAGENTA if elbow_frozen else FG),
            (font, "", FG),
            (big, "HAND  (computed)", AMBER),
            (font, f"   x = {hx:8.4f} m", FG),
            (font, f"   y = {hy:8.4f} m", FG),
            (font, f"   r = {math.hypot(hx, hy):8.4f} m", DIM),
        ]
        y = panel.top + 14
        for f, text, colour in lines:
            screen.blit(f.render(text, True, colour), (panel.left + 14, y))
            y += 28

        status = [
            (f"limits {'ON' if limits_on else 'OFF'}  [L]",
             LIME if limits_on else DIM),
            (f"elbow {'FROZEN' if elbow_frozen else 'free'}  [F]",
             MAGENTA if elbow_frozen else DIM),
            (f"{len(trail)} points painted   [C] clear  [R] reset", DIM),
        ]
        y = 300
        for text, colour in status:
            screen.blit(font.render(text, True, colour), (800, y))
            y += 26

        screen.blit(font.render("W / S  shoulder      O / K  elbow", True, FG),
                    (30, H - 62))
        screen.blit(
            font.render("nothing here was measured", True, DIM),
            (30, H - 34))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()