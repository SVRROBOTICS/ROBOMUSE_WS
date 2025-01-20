#!/usr/bin/env python3

import logging
from utils import calculate_motor_speeds, encoder_ticks_to_velocity

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

WHEEL_DIA = 0.0625 # in meters
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

    def __init__(self, parent_node):
        # Store the parent node instance
        self.parent_node = parent_node
        self.cmd_vel = None  # Initialize cmd_vel attribute
        time_delta = 1/20
        
        # Initialize PID controllers for both wheels
        self.pid1 = PIDController(kp=0.1, ki=0.0, kd=0.00, dt=time_delta)
        self.pid2 = PIDController(kp=0.11, ki=0.0, kd=0.00, dt=time_delta)
        
        # Previous encoder values to compute velocity
        self.prev_encoder1 = self.parent_node.robot.get_encoder1()
        self.prev_encoder2 = self.parent_node.robot.get_encoder2()

    def get_encoder1(self):
        # This function should return the current encoder value for wheel 1
        return self.parent_node.robot.get_encoder1()

    def get_encoder2(self):
        # This function should return the current encoder value for wheel 2
        return self.parent_node.robot.get_encoder2()

    def calculate_velocity(self, current_encoder, prev_encoder, dt):
        # Calculate the velocity based on encoder change and time step
        delta_position = current_encoder - prev_encoder
        velocity = encoder_ticks_to_velocity(delta_position, dt)
        return velocity
    

    def compute_pid_control(self, command_vel, current_encoder, prev_encoder, pid_controller):
        # Compute the actual velocity from encoder readings
        actual_velocity = self.calculate_velocity(current_encoder, prev_encoder, pid_controller.dt)
        
        # Compute the error between the desired velocity and the actual velocity
        error = command_vel - actual_velocity
        
        # Use PID controller to compute the new control signal (wheel speed command)
        wheel_speed_cmd = pid_controller.update(error)
        
        return wheel_speed_cmd


    def control_motors(self, encoder1, encoder2, cmd_vel):
        """
        Calculate and return motor speed commands using PID control.
        :param encoder1: Current encoder reading for motor 1
        :param encoder2: Current encoder reading for motor 2
        :param cmd_vel: Commanded velocities (linear and angular)
        :return: Speed commands for both motors
        """
        if cmd_vel is None:
            # No velocity command provided
            return None, None
        
        # Calculate desired left and right wheel velocities
        command_left_velocity, command_right_velocity = calculate_motor_speeds(cmd_vel.linear.x, cmd_vel.angular.z)
        logger.info(f"command_left_velocity Value: {command_left_velocity}, command_right_velocity Value: {command_right_velocity}")

        # Calculate the wheel speed commands using PID control
        speed_cmd1 = self.compute_pid_control(command_left_velocity, encoder1, self.prev_encoder1, self.pid1)
        speed_cmd2 = self.compute_pid_control(command_right_velocity, encoder2, self.prev_encoder2, self.pid2)
        
        # Update previous encoder values for the next iteration
        self.prev_encoder1 = encoder1
        self.prev_encoder2 = encoder2
        
        # Return the calculated speed commands for both motors
        return speed_cmd1, speed_cmd2
    
    def compute_pid_control(self, command_vel, current_encoder, prev_encoder, pid_controller):
        # Compute the actual velocity from encoder readings
        actual_velocity = self.calculate_velocity(current_encoder, prev_encoder, pid_controller.dt)
        
        # Compute the error between the desired velocity and the actual velocity
        error = command_vel - actual_velocity
        
        # Use PID controller to compute the new control signal (wheel speed command)
        wheel_speed_cmd = pid_controller.update(error)
        
        return wheel_speed_cmd
    










