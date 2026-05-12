# Vision Robotic CTF

A real-time robotic Capture the Flag pursuit-evasion game using vision-based autonomous agents on the RoboMaster platform.

## Overview

This project simulates and implements a robotic Capture the Flag game with two mobile agents: an attacker and a defender. The attacker must retrieve a centrally located flag and deliver it to a drop zone, while the defender tracks and intercepts the attacker using vision-based detection.

The system combines grid-based path planning, real-time visual tracking, and multi-threaded robot control to model autonomous decision-making in an adversarial robotics environment.

## Features

- RoboMaster-based robotic Capture the Flag setup
- 3x3 grid arena simulation
- Attacker and defender agent behavior
- HSV-based color segmentation for visual tracking
- A* path planning for navigation
- Dynamic rerouting during pursuit-evasion
- Multi-threaded control architecture
- Python simulation with randomized obstacle placement

## Files

- `attacker.py` - Attacker robot logic
- `defender.py` - Defender robot logic
- `defender 2.py` - Alternate defender implementation
- `robo_simulation.py` - Python simulation environment
- `JDS_Final_Report (1).pdf` - Final project report

## Project Title

**Capture the Flag: A Real-Time Robotic Pursuit-Evasion Game Using Vision-Based Autonomous Agents**

## Authors

Arooba Farhan, Shariq Kazmi, and Mahnoor Kashif  
Lahore University of Management Sciences, Lahore, Pakistan

## Technologies Used

- Python
- RoboMaster SDK
- OpenCV
- HSV color segmentation
- A* path planning
- Multi-threading

## Description

The attacker navigates toward the flag and then to the drop zone while avoiding the defender. The defender uses camera-based perception to detect and track the attacker in real time. Both agents use autonomous behavior pipelines that combine perception, planning, and motion control.

The simulation models strategic interactions between agents, while the hardware implementation translates the same ideas to physical RoboMaster robots in a controlled arena.

## License

This project was developed for academic purposes.
