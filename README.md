# Reaction-Wheel Inverted Pendulum Simulation

A physical control system featuring a 3D Reaction-Wheel Inverted Pendulum modeled in CoppeliaSim and dynamically balanced via an external Python control loop using the ZeroMQ Remote API.

## Project Overview
* **Simulation Engine:** CoppeliaSim Edu
* **Control Architecture:** Proportional-Integral-Derivative (PID) feedback loop
* **Communication:** Real-time IPC via ZeroMQ Remote API
* **Target:** Maintain active vertical balance ($0^\circ$ setpoint) under dynamic disturbances.

## Repository Structure
* `rwip_control.py`: Main Python control loop executing PID gains and ZeroMQ joint commands.
* `Reaction_Wheel_Pendulum.ttt`: CoppeliaSim 3D scene file including kinematics and dynamic body mass properties.

## How to Run
1. Open `Reaction_Wheel_Pendulum.ttt` inside CoppeliaSim.
2. Install dependencies: `pip install coppeliasim-zmqremoteapi-client`
3. Execute the control script: `python3 rwip_control.py`
