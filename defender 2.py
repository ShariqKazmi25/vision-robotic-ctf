defender code:


#!/usr/bin/env python3
"""
defender_physical_ctf_realsense.py — Final Defender on Jetson + RealSense

• 4×4 grid, 0.9 m cells  
• Intel RealSense D45 for color+depth  
• Initial “wide‐view” shortest‐turn toward ATK_START  
• Play bravo.wav on EP Core speaker → attacker “go”  
• Detect 14×14 cm red cube: HSV+high-S/V mask, R-dominance filter, depth gating, aspect-ratio & size matching  
• Compute distance + bearing, estimate attacker (x,y) & cell  
• A* intercept next step toward flag  
• 0.7 m/s drive, 60 °/s rotate (EP Core’s +z is CW)  
• 150 s time limit  
• Continuously prints DEF & ATK positions + headings  
"""
import time, math, heapq
from queue import Empty

import cv2, numpy as np, pyrealsense2 as rs
from robomaster import robot

# ─── GRID & GAME POINTS ──────────────────────────────────────────
GRID_DIM   = 4
CELL_M     = 0.9
FLAG_PT    = (0, 0)
DROP_PT    = (3, 3)
DEF_START  = (3, 0)
ATK_START  = (2, 3)
START_HEAD = 90.0    # degrees, 0° = +X, CCW+
TIME_LIMIT = 150.0   # seconds

# ─── CAMERA & MASK CONFIG ────────────────────────────────────────
IMG_W, IMG_H = 1280, 720
H_FOV_DEG    = 90.0
FOCAL_PIX    = (IMG_W/2) / math.tan(math.radians(H_FOV_DEG/2))
DEG_PER_PX   = H_FOV_DEG / IMG_W

# red-cube HSV bounds: very high S & V to cut out brown/orange
R_LO1, R_UP1 = np.array([0, 150, 150]),   np.array([8, 255, 255])
R_LO2, R_UP2 = np.array([170,150,150]),   np.array([180,255,255])
MIN_RED_A    = 500    # px²
TARGET_FACE  = 0.14   # m

# depth gating
DEPTH_MIN    = 0.07   # m
DEPTH_MAX    = 5.0    # m

K_OPEN       = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
K_CLOSE      = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))

# drive params
DRIVE_SPEED  = 0.7    # m/s
ROT_SPEED    = 60     # °/s
CELL_TIME    = CELL_M/DRIVE_SPEED + 0.2

def wrap180(a): return ((a+180)%360)-180

def cell_to_xy(cell):
    r, c = cell
    x = c * CELL_M
    y = (GRID_DIM - 1 - r) * CELL_M
    return x, y

def astar(start, goal):
    steps = [(-1,0),(1,0),(0,-1),(0,1)]
    open_set = [(0, start)]
    came, g, f = {}, {start:0}, {start:abs(start[0]-goal[0])+abs(start[1]-goal[1])}
    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]; path.append(cur)
            return path[::-1]
        for dx,dy in steps:
            nb = (cur[0]+dx, cur[1]+dy)
            if not (0<=nb[0]<GRID_DIM and 0<=nb[1]<GRID_DIM): continue
            tg = g[cur]+1
            if tg < g.get(nb,1e9):
                came[nb] = cur
                g[nb] = tg
                f[nb] = tg + abs(nb[0]-goal[0])+abs(nb[1]-goal[1])
                heapq.heappush(open_set, (f[nb], nb))
    return None

STEP2DR = {
    (-1,0):( DRIVE_SPEED, 0),
    ( 1,0):(-DRIVE_SPEED,0),
    ( 0,1):(0, DRIVE_SPEED),
    ( 0,-1):(0,-DRIVE_SPEED)
}

def detect_red_cube(frame, depth_frame):
    """
    Robust detector returns (cx, cy, dist, bear, mask) or None.
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m1   = cv2.inRange(hsv, R_LO1, R_UP1)
    m2   = cv2.inRange(hsv, R_LO2, R_UP2)
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  K_OPEN)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, K_CLOSE)

    # R-dominance filter to reject brown/orange background
    b,g,r = cv2.split(frame)
    dom = ((r.astype(int)-g.astype(int)>50) & (r.astype(int)-b.astype(int)>50))
    mask = cv2.bitwise_and(mask, mask, mask=dom.astype(np.uint8)*255)

    best_score = 1e9
    best_blob  = None

    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        if cv2.contourArea(cnt) < MIN_RED_A: continue

        x,y,w,h = cv2.boundingRect(cnt)
        ar = w/h if h>0 else 0
        if not (0.75 < ar < 1.25):    # roughly square
            continue

        cx, cy = x + w/2, y + h/2
        ix, iy = int(cx), int(cy)
        dist = depth_frame.get_distance(ix, iy)
        if not (DEPTH_MIN < dist < DEPTH_MAX):
            continue

        real_w = (w * dist) / FOCAL_PIX
        real_h = (h * dist) / FOCAL_PIX
        size_err = abs(real_w - TARGET_FACE) + abs(real_h - TARGET_FACE)
        tol = 0.03 + 0.02*dist      # dynamic tol
        if size_err > tol*2:
            continue

        # score = size_err + aspect‐error
        score = size_err + abs(ar - 1.0)
        if score < best_score:
            best_score = score
            bear = (cx - IMG_W/2)*DEG_PER_PX
            best_blob = (ix, iy, dist, bear, mask)

    return best_blob

def main():
    # connect EP → RNDIS
    ep      = robot.Robot(); ep.initialize(conn_type="rndis")
    chassis = ep.chassis

    # start RealSense
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.depth, IMG_W, IMG_H, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 30)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    # init
    dc = DEF_START
    th = math.radians(START_HEAD)
    print(f"DEF start={dc}@{START_HEAD:.1f}°  ATK start={ATK_START}")
    print(f"FLAG={FLAG_PT}  DROP={DROP_PT}")

    # initial wide‐view turn
    xa,ya = cell_to_xy(ATK_START)
    xd,yd = cell_to_xy(dc)
    tgt   = math.degrees(math.atan2(ya-yd, xa-xd))
    δ0    = wrap180(tgt - math.degrees(th))
    slack = H_FOV_DEG/4.0
    δ1    = δ0 - slack if δ0>0 else δ0 + slack
    δ1    = wrap180(δ1)
    zcmd  = -δ1
    print(f"→ turning EP by {zcmd:+.1f}° (toward {tgt:+.1f}° ±{slack:.1f})")
    chassis.move(x=0, y=0, z=zcmd, z_speed=ROT_SPEED).wait_for_completed()
    th += math.radians(-zcmd)

    # beep → attacker go
    print("…waiting for attacker; will beep once seen")
    try:
        ep.audio.set_volume(30)
        ep.audio.play_wav('/home/root/bravo.wav')
    except Exception as e:
        print("⚠ speaker:", e)
    print('\a', end='', flush=True)

    # wait for first detection
    start_time = None
    while True:
        frames = pipeline.wait_for_frames()
        aligned= align.process(frames)
        dfrm   = aligned.get_depth_frame()
        cfrm   = aligned.get_color_frame()
        if not (dfrm and cfrm): continue
        img  = np.asanyarray(cfrm.get_data())
        det  = detect_red_cube(img, dfrm)
        if det:
            print("attacker detected → starting timer")
            start_time = time.time()
            break

    last_side = None

    # main loop
    while True:
        if start_time and (time.time()-start_time) > TIME_LIMIT:
            print("⏰ time up"); break

        frames = pipeline.wait_for_frames()
        aligned= align.process(frames)
        dfrm   = aligned.get_depth_frame()
        cfrm   = aligned.get_color_frame()
        if not (dfrm and cfrm): continue
        img  = np.asanyarray(cfrm.get_data())
        det  = detect_red_cube(img, dfrm)

        if not det:
            spin = -ROT_SPEED/2 if last_side=='left' else ROT_SPEED/2 if last_side=='right' else ROT_SPEED/4
            chassis.drive_speed(0,0,spin)
            print("…lost view, spinning to reacquire")
            continue

        ix,iy,dist,bear,mask = det
        chassis.drive_speed(0,0,0)
        last_side = 'left' if bear < -5 else 'right' if bear > 5 else None

        # center cube if off-center
        if abs(bear) > 5.0:
            print(f"Centering target (bearing {bear:+.1f}°)…")
            zcmd = -bear
            chassis.move(x=0, y=0, z=zcmd, z_speed=ROT_SPEED).wait_for_completed()
            th += math.radians(-zcmd)
            continue

        # compute attacker cell
        xdf, ydf = cell_to_xy(dc)
        ax = xdf + dist*math.cos(math.radians(bear)+th)
        ay = ydf + dist*math.sin(math.radians(bear)+th)
        cell_r = GRID_DIM-1 - int(round(ay/CELL_M))
        cell_c = int(round(ax/CELL_M))
        ac = ( max(0,min(GRID_DIM-1,cell_r)),
               max(0,min(GRID_DIM-1,cell_c)) )

        print(f"DEF_xy=({xdf:.2f},{ydf:.2f})@{math.degrees(th):.1f}°  "
              f"ATK_xy=({ax:.2f},{ay:.2f}) cell={ac}  dist={dist:.2f} bear={bear:+.1f}")

        # intercept
        pA  = astar(ac, FLAG_PT)
        nxt = pA[1] if pA and len(pA)>1 else ac
        pD  = astar(dc, nxt)
        if not pD or len(pD)<2:
            print("⏸ no path → holding"); chassis.drive_speed(0,0,0); time.sleep(0.5); continue

        nxtD = pD[1]
        step = (nxtD[0]-dc[0], nxtD[1]-dc[1])
        drv  = STEP2DR.get(step)
        print(f"→ DEF moves {step}")
        chassis.drive_speed(x=drv[0],y=drv[1],z=0)
        time.sleep(CELL_TIME)
        chassis.drive_speed(0,0,0)
        dc = nxtD

    # cleanup
    chassis.drive_speed(0,0,0)
    pipeline.stop()
    ep.close()

if _name=="main_":
    main()