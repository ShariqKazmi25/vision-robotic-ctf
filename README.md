# RoboMaster EP Core Capture the Flag

A LUMS robotics project that paired a keyboard-controlled DJI RoboMaster attacker with an autonomous defender in an indoor capture-the-flag demo. The defender used a Jetson Nano, an Intel RealSense RGB-D camera, OpenCV, and the RoboMaster SDK.

## Physical demo

The defender used HSV color segmentation to detect a red marker on the attacker. Aligned depth measurements and the marker's image position provided range and bearing for estimating the attacker's grid cell. The defender replanned its next move with A* and searched for the marker when it went out of view. The [project report](JDS_Final_Report%20%281%29.pdf) documents the indoor trial and discusses camera field-of-view limits and false positives from background colors.

## Run the simulation

The `10 × 10` Python simulation has a fixed obstacle and a randomly moving patrol. Choose one role at the prompt and enter directions in the terminal. The computer controls the other role with A* and Matplotlib displays the arena.

```bash
python -m pip install numpy matplotlib
python robo_simulation.py
```

## Repository contents

| File | Contents |
| --- | --- |
| [`robo_simulation.py`](robo_simulation.py) | Interactive grid simulation for one human player and one computer-controlled agent |
| [`attacker.py`](attacker.py) | Keyboard teleoperation prototype using the RoboMaster SDK |
| [`defender.py`](defender.py) | RealSense marker-detection and range/bearing prototype |
| [`defender 2.py`](defender%202.py) | Integrated defender prototype with RGB-D tracking, A* planning, and chassis control |

The hardware scripts are archived lab prototypes. Running them on a fresh checkout requires cleaning up formatting and entry points and adapting the settings to the robot, camera, and arena.

## Authors

Arooba Farhan, Shariq Kazmi, and Mahnoor Kashif, Lahore University of Management Sciences.
