#!/usr/bin/env python3
from utils import calculate_motor_speeds
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class RobotController:
    def __init__(self):
        # Define PID controllers for both wheels with some arbitrary gains for demonstration.
        self.pid1 = PIDController(kp=1.0, ki=0.1, kd=0.01, dt=0.1)
        self.pid2 = PIDController(kp=1.0, ki=0.1, kd=0.01, dt=0.1)
        
        # Previous encoder values to compute velocity (we store these to compute velocity from encoder readings)
        self.prev_encoder1 = 0
        self.prev_encoder2 = 0
        
    def get_encoder1(self):
        # This function should return the current encoder value for wheel 1
        return self.robot.get_encoder1()

    def get_encoder2(self):
        # This function should return the current encoder value for wheel 2
        return self.robot.get_encoder2()

    def calculate_velocity(self, current_encoder, prev_encoder):
        # Calculate the velocity based on encoder change and time step
        # Assuming encoder value is in counts, and we compute velocity as a simple difference
        # This needs to be adjusted based on how encoder values are read (e.g., time between reads)
        delta_position = current_encoder - prev_encoder
        velocity = delta_position / self.dt  # Velocity in counts per second
        return velocity

    def compute_pid_control(self, command_vel, current_encoder, prev_encoder, pid_controller):
        # Compute the actual velocity from encoder readings
        actual_velocity = self.calculate_velocity(current_encoder, prev_encoder)
        
        # Compute the error between the desired velocity and the actual velocity
        error = command_vel - actual_velocity
        
        # Use PID controller to compute the new control signal (wheel speed command)
        wheel_speed_cmd = pid_controller.update(error)
        
        return wheel_speed_cmd

    def control_motors(self, encoder1, encoder2):

        cmd_vel = self.cmd_vel

        if cmd_vel is None:
            self.get_logger().info(f"No cmd_vel command for robot available.")
            return None, None
        
        command_left_velocity, command_right_velocity = calculate_motor_speeds(cmd_vel.linear.x, cmd_vel.angular.z)

        # Calculate the wheel speed commands using PID control
        speed_cmd1 = self.compute_pid_control(command_left_velocity, encoder1, self.prev_encoder1, self.pid1)
        speed_cmd2 = self.compute_pid_control(command_right_velocity, encoder2, self.prev_encoder2, self.pid2)
        
        # Update previous encoder values for next iteration
        self.prev_encoder1 = encoder1
        self.prev_encoder2 = encoder2
        
        # Return the calculated speed commands for both motors
        return speed_cmd1, speed_cmd2

