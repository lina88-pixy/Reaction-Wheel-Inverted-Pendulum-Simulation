import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, output_limits=(-10.0, 10.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output, self.max_output = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, setpoint: float, current_value: float) -> float:
        current_time = time.time()
        if self.last_time is None:
            dt = 0.01
        else:
            dt = current_time - self.last_time
            dt = max(dt, 0.001)  # Avoid division by zero

        # Error calculation (Angle error from vertical 0.0 rad)
        error = setpoint - current_value

        # P, I, D Terms
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Total command output
        output = p_term + i_term + d_term

        # Clamp output to motor physical limits
        output = max(self.min_output, min(self.max_output, output))

        self.prev_error = error
        self.last_time = current_time

        return output

def main():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')

    try:
        # Absolute scene paths based on your CoppeliaSim hierarchy:
        pendulum_joint = sim.getObject('/Stand/pendulum_joint')
        wheel_motor = sim.getObject('/Stand/pendulum_joint/PendulumBody/wheel_motor')
    except Exception as e:
        print("\n[ERROR] Could not find object handles!")
        print("Make sure your joint names in CoppeliaSim match '/ReactionWheel_Joint' and '/Pendulum_Base'.")
        return

    pid = PIDController(kp=15.0, ki=0.0, kd=1.5, output_limits=(-10.0, 10.0))
    target_angle = 0.0  # Desired vertical angle in radians

    sim.setStepping(True)
    sim.startSimulation()

    print("Simulation started! Press Ctrl+C in terminal to stop.")

    try:
        while True:
            # 1. Read current tilt angle of the pendulum joint (in radians)
            current_angle = sim.getJointPosition(pendulum_joint)

            # 2. Compute required reaction torque via PID
            torque = pid.compute(setpoint=target_angle, current_value=current_angle)

            # 3. Apply torque to the reaction wheel motor
            sim.setJointTargetForce(wheel_motor, abs(torque))
            # Set high target velocity so force/torque dictates the acceleration
            sim.setJointTargetVelocity(wheel_motor, math.copysign(99999, torque))

            # Step physics forward by 1 frame
            sim.step()

    except KeyboardInterrupt:
        print("\nStopping simulation...")
    finally:
        sim.stopSimulation()
        print("Simulation stopped cleanly.")

if __name__ == "__main__":
    main()