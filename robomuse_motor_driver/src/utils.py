#!/usr/bin/env python3

import math
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

def encoder_ticks_to_velocity(delta_ticks, delta_time):
        """
        Convert encoder ticks to velocity in meters per second.
        
        :param delta_ticks: Change in encoder ticks
        :param tpr: Ticks per revolution of the encoder
        :param wheel_radius: Radius of the wheel in meters
        :param delta_time: Time interval in seconds
        :return: Velocity in meters per second
        """
        if delta_time == 0:
            raise ValueError("Delta time cannot be zero.")
        tpr = 200550
        # Calculate the distance traveled in meters
        distance_traveled = (delta_ticks / tpr) * (math.pi * WHEEL_DIA)
        
        # Calculate velocity in meters per second
        velocity = distance_traveled / delta_time
        
        return velocity
