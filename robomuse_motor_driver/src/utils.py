#!/usr/bin/env python3

def calculate_motor_speeds(linear, angular):
        """
        Convert linear and angular velocity to left and right motor speeds.
        Adjust the formula based on your motor setup and robot geometry.
        """
        wheel_base = 0.5  # Distance between wheels in meters (adjust as per your robot)
        
        # Differential drive formula
        left_motor_speed = linear - (angular * wheel_base / 2.0)
        right_motor_speed = linear + (angular * wheel_base / 2.0)
        
        return left_motor_speed, right_motor_speed
