from math import atan

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle


    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;



'''
  <param name="vehicle_mass" value="1080." />
<param name="fuel_capacity" value="0." />
<param name="brake_deadband" value=".2" />
<param name="decel_limit" value="-5." />
<param name="accel_limit" value="1." />
<param name="wheel_radius" value="0.335" />
<param name="wheel_base" value="3" />
<param name="steer_ratio" value="2.67" />
<param name="max_lat_accel" value="3." />
<param name="max_steer_angle" value="8." />

'''
