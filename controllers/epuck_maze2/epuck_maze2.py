"""
E-puck maze controller — Tasks 2 & 3
Auto-calibrates sensor thresholds at startup.
Left-hand wall following with timed turns (no per-step oscillation).
"""
from controller import Robot
import random, math

TIME_STEP   = 64
MAX_SPEED   = 6.28
TURN_SPEED  = MAX_SPEED * 0.5
FWD_SPEED   = MAX_SPEED * 0.8

CALIBRATION_STEPS = 20   # steps without moving to measure background
DELTA_SIDE  = 5          # wall detected when sensor > median + DELTA
DELTA_FRONT = 9          # higher margin for diagonal front sensors

TURN_STEPS    = 14   # ~90° at TURN_SPEED
FORWARD_STEPS = 6    # straight after a left turn (cross the junction)

GREEN_THR    = 50
COMM_CHANNEL = 1
STUCK_STEPS  = 100
LEADER_THR   = 500  # sensor delta that means another robot is very close ahead (~5 cm)

PS_FR = 0; PS_RF = 1; PS_R = 2; PS_RB = 3
PS_B  = 4; PS_LB = 5; PS_L = 6; PS_FL = 7


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
    robot  = Robot()
    name   = robot.getName()
    my_id  = 1 if name == "e-puck" else 2

    lm = robot.getDevice("left wheel motor")
    rm = robot.getDevice("right wheel motor")
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0);            rm.setVelocity(0)

    ps = [robot.getDevice(f"ps{i}") for i in range(8)]
    for s in ps:
        s.enable(TIME_STEP)

    camera = robot.getDevice("camera")
    camera.enable(TIME_STEP)

    # Use built-in emitter/receiver (E-puck PROTO includes them)
    emitter  = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    receiver.enable(TIME_STEP)
    emitter.setChannel(COMM_CHANNEL)
    receiver.setChannel(COMM_CHANNEL)

    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)

    # ── Calibration: stand still, measure open-space baseline ─────────────────
    cal_readings = []
    for _ in range(CALIBRATION_STEPS):
        robot.step(TIME_STEP)
        cal_readings.extend(s.getValue() for s in ps)

    cal_readings.sort()
    baseline = cal_readings[len(cal_readings) // 2]   # median, robust to outliers

    SIDE_THR  = baseline + DELTA_SIDE
    FRONT_THR = baseline + DELTA_FRONT
    print(f"[{name}] calibrated: baseline={baseline:.1f} "
          f"SIDE_THR={SIDE_THR:.1f} FRONT_THR={FRONT_THR:.1f}")

    # ── State ─────────────────────────────────────────────────────────────────
    nav   = "follow"     # follow | turn_right | turn_left | forward_after_left | done
    tc    = 0            # turn counter
    fc    = 0            # forward counter
    step  = 0

    comm_state = "solo"
    is_leader  = False
    my_roll    = None
    stuck_cnt  = 0
    last_xy    = None

    while robot.step(TIME_STEP) != -1:
        pv  = [s.getValue() for s in ps]
        pos = gps.getValues()
        step += 1

        # ── Debug every 30 steps ──────────────────────────────────────────────
        if step % 30 == 1:
            print(f"[{name}] step={step} nav={nav} comm={comm_state} "
                  f"ps={[int(v) for v in pv]}")

        # ── Exit detection (every 5 steps) ────────────────────────────────────
        if nav != "done" and step % 5 == 0:
            gf = green_fraction(camera)
            if gf > 0.05:
                print(f"[{name}] exit gf={gf:.2f}")
                if gf > 0.35:
                    print(f"[{name}] REACHED EXIT")
                    lm.setVelocity(0); rm.setVelocity(0)
                    emitter.send(f"FOUND_EXIT:{my_id}".encode())
                    nav = "done"
                    continue
                lm.setVelocity(FWD_SPEED); rm.setVelocity(FWD_SPEED)
                continue

        if nav == "done":
            lm.setVelocity(0); rm.setVelocity(0)
            continue

        # ── Communication ─────────────────────────────────────────────────────
        while receiver.getQueueLength() > 0:
            msg = receiver.getString()
            receiver.nextPacket()
            if msg.startswith("HELLO:") and comm_state == "solo":
                my_roll = random.random()
                emitter.send(f"ROLL:{my_id}:{my_roll:.6f}".encode())
                comm_state = "negotiating"
            elif msg.startswith("ROLL:") and comm_state in ("solo", "negotiating"):
                other_roll = float(msg.split(":")[2])
                if my_roll is None:
                    my_roll = random.random()
                    emitter.send(f"ROLL:{my_id}:{my_roll:.6f}".encode())
                is_leader  = my_roll >= other_roll
                comm_state = "leader" if is_leader else "follower"
                stuck_cnt  = 0
                print(f"[{name}] -> {'LEADER' if is_leader else 'FOLLOWER'}")
            elif msg == "SWAP_ROLES" and comm_state in ("leader", "follower"):
                is_leader  = not is_leader
                comm_state = "leader" if is_leader else "follower"
                stuck_cnt  = 0

        if comm_state == "solo":
            if any(pv[i] > SIDE_THR + 150 for i in (PS_FR, PS_FL)):
                my_roll = random.random()
                emitter.send(f"HELLO:{my_id}".encode())
                emitter.send(f"ROLL:{my_id}:{my_roll:.6f}".encode())
                comm_state = "negotiating"

        # Stuck detection (leader)
        if comm_state == "leader" and last_xy is not None:
            dx = pos[0]-last_xy[0]; dy = pos[1]-last_xy[1]
            stuck_cnt = stuck_cnt+1 if math.sqrt(dx*dx+dy*dy) < 0.001 else 0
            if stuck_cnt > STUCK_STEPS:
                emitter.send(b"SWAP_ROLES")
                is_leader = False; comm_state = "follower"; stuck_cnt = 0
        last_xy = (pos[0], pos[1])

        # ── Follower / negotiating ────────────────────────────────────────────
        if comm_state == "negotiating":
            lm.setVelocity(0); rm.setVelocity(0)
            continue
        if comm_state == "follower":
            # Stop only when leader robot is very close directly ahead
            leader_close = (pv[PS_FR] - baseline > LEADER_THR or
                            pv[PS_FL] - baseline > LEADER_THR)
            if leader_close:
                lm.setVelocity(0); rm.setVelocity(0)
                continue
            # Otherwise navigate the maze independently (same as solo)

        # ── Wall detection (adaptive: above calibrated background) ────────────
        w_front = (pv[PS_FR] > FRONT_THR or pv[PS_FL] > FRONT_THR)
        w_left  =  pv[PS_L]  > SIDE_THR
        w_right =  pv[PS_R]  > SIDE_THR

        # ── Navigation state machine ──────────────────────────────────────────
        if nav == "turn_right":
            lm.setVelocity(TURN_SPEED); rm.setVelocity(-TURN_SPEED)
            tc += 1
            if tc >= TURN_STEPS:
                nav = "follow"; tc = 0

        elif nav == "turn_left":
            lm.setVelocity(-TURN_SPEED); rm.setVelocity(TURN_SPEED)
            tc += 1
            if tc >= TURN_STEPS:
                # After left turn: go forward briefly before checking sensors
                nav = "forward_after_left"; tc = 0; fc = 0

        elif nav == "forward_after_left":
            lm.setVelocity(FWD_SPEED); rm.setVelocity(FWD_SPEED)
            fc += 1
            if fc >= FORWARD_STEPS or w_front:
                nav = "follow"; fc = 0

        else:  # "follow" — left-hand rule
            if not w_left:
                nav = "turn_left"; tc = 0
                lm.setVelocity(-TURN_SPEED); rm.setVelocity(TURN_SPEED)
            elif not w_front:
                lm.setVelocity(FWD_SPEED); rm.setVelocity(FWD_SPEED)
            elif not w_right:
                nav = "turn_right"; tc = 0
                lm.setVelocity(TURN_SPEED); rm.setVelocity(-TURN_SPEED)
            else:
                nav = "turn_right"; tc = 0
                lm.setVelocity(TURN_SPEED); rm.setVelocity(-TURN_SPEED)

    while robot.step(TIME_STEP) != -1:
        lm.setVelocity(0); rm.setVelocity(0)


if __name__ == "__main__":
    main()
