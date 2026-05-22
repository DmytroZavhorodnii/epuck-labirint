"""
E-puck maze controller — Tasks 1 & 2 (single robot)
Strategy: left-hand wall following + green-colour camera exit detection.
Coordinate system: Webots R2023b Z-up (ENU).
"""
from controller import Robot

MAX_SPEED  = 6.28
TURN_SPEED = MAX_SPEED * 0.45
HALF_SPEED = MAX_SPEED * 0.5
TIME_STEP  = 64

# ps0/ps7 are diagonal (45°) — they see side walls at angle, so use higher
# threshold to avoid false "front wall" when a side wall is near.
FRONT_THR = 300   # front sensors (ps0, ps7)
SIDE_THR  = 80    # side sensors (ps1, ps2, ps5, ps6)
GREEN_THR = 50    # green channel dominance to detect exit marker

# E-puck ps indices (clockwise from front-right when viewed from above)
PS_FRONT_R = 0   #  10 deg  (front, slight right)
PS_FRONT_L = 7   # 350 deg  (front, slight left — 315° CW from front)
PS_RIGHT_F = 1   #  45 deg
PS_RIGHT   = 2   #  90 deg
PS_LEFT_B  = 5   # 225 deg
PS_LEFT    = 6   # 270 deg


def green_fraction(camera):
    img = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    total = w * h
    if total == 0:
        return 0.0
    count = 0
    for y in range(h):
        for x in range(w):
            g = camera.imageGetGreen(img, w, x, y)
            if (g > camera.imageGetRed(img, w, x, y) + GREEN_THR and
                    g > camera.imageGetBlue(img, w, x, y) + GREEN_THR):
                count += 1
    return count / total


def main():
    robot = Robot()

    lm = robot.getDevice("left wheel motor")
    rm = robot.getDevice("right wheel motor")
    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    lm.setVelocity(0)
    rm.setVelocity(0)

    ps = [robot.getDevice(f"ps{i}") for i in range(8)]
    for s in ps:
        s.enable(TIME_STEP)

    camera = robot.getDevice("camera")
    camera.enable(TIME_STEP)

    print("[epuck_maze] Started — searching for exit...")

    found = False
    while robot.step(TIME_STEP) != -1 and not found:
        pv = [s.getValue() for s in ps]

        w_front = pv[PS_FRONT_R] > FRONT_THR or pv[PS_FRONT_L] > FRONT_THR
        w_left  = pv[PS_LEFT]    > SIDE_THR  or pv[PS_LEFT_B]  > SIDE_THR
        w_right = pv[PS_RIGHT]   > SIDE_THR  or pv[PS_RIGHT_F] > SIDE_THR

        gf = green_fraction(camera)
        if gf > 0.05:
            print(f"[epuck_maze] Exit visible (green={gf:.2f})")
            if gf > 0.40:
                print("[epuck_maze] Reached exit!")
                lm.setVelocity(0)
                rm.setVelocity(0)
                found = True
            else:
                lm.setVelocity(HALF_SPEED)
                rm.setVelocity(HALF_SPEED)
            continue

        # Left-hand wall following
        if not w_left:
            lm.setVelocity(-TURN_SPEED)
            rm.setVelocity(TURN_SPEED)
        elif not w_front:
            lm.setVelocity(MAX_SPEED)
            rm.setVelocity(MAX_SPEED)
        elif not w_right:
            lm.setVelocity(TURN_SPEED)
            rm.setVelocity(-TURN_SPEED)
        else:
            lm.setVelocity(MAX_SPEED)
            rm.setVelocity(-MAX_SPEED)

    while robot.step(TIME_STEP) != -1:
        lm.setVelocity(0)
        rm.setVelocity(0)


if __name__ == "__main__":
    main()
