#!/usr/bin/env python3
"""
teleop_attacker.py

Teleoperation script for DJI RoboMaster EP Core attacker robot.
Controls via keyboard:
  W/S = forward/backward
  A/D = strafe left/right
  Q/E = rotate left/right
Press 'Ctrl+C' to exit.

Dependencies:
    pip install robomaster
"""

import sys
import time
import os

from robomaster import robot

# movement speeds
LIN_SPEED   = 0.5    # m/s for forward/back/strafe
ANG_SPEED   = 100    # deg/s for yaw

# key→(x, y, z) axes: x=forward+, y=left+, z=yaw right+
KEY_BINDINGS = {
    'w': ( 1,  0,  0),
    's': (-1,  0,  0),
    'a': ( 0,  1,  0),
    'd': ( 0, -1,  0),
    'q': ( 0,  0,  1),
    'e': ( 0,  0, -1),
}

# platform-specific get_key
if os.name == 'nt':
    import msvcrt

    def get_key(timeout=0.1):
        start = time.time()
        while time.time() - start < timeout:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                return ch.lower()
        return ''
else:
    import tty
    import termios
    import select

    def get_key(timeout=0.1):
        dr, _, _ = select.select([sys.stdin], [], [], timeout)
        if dr:
            return sys.stdin.read(1).lower()
        return ''

def main():
    # On Unix, switch stdin to raw mode
    if os.name != 'nt':
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    chassis = ep_robot.chassis

    print("Teleop attacker: W/S/A/D = move, Q/E = turn, Ctrl+C to quit.")

    try:
        while True:
            key = get_key(0.05)
            x = y = z = 0
            if key in KEY_BINDINGS:
                dx, dy, dz = KEY_BINDINGS[key]
                x = dx * LIN_SPEED
                y = dy * LIN_SPEED
                z = dz * ANG_SPEED

            # send RC command every cycle
            chassis.drive_rc(
                x=x, y=y, z=z,
                x_speed=LIN_SPEED,
                y_speed=LIN_SPEED,
                z_speed=ANG_SPEED
            )
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        # stop movement
        chassis.drive_rc(x=0, y=0, z=0,
                         x_speed=0, y_speed=0, z_speed=0)
        ep_robot.close()
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        print("\nTeleop terminated.")

if _name_ == "_main_":
    main()