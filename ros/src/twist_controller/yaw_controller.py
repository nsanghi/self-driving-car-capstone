import math


class YawController(object):

    def __init__(self, wheel_base, steer_ratio):
        self.wb = wheel_base
        self.sr = steer_ratio


    def get_steering(self, lin_vel, ang_vel):
        ang_vel = ang_vel if abs(lin_vel) > 0.0 else 0.0
        return self.sr * math.atan(ang_vel * self.wb / lin_vel) if abs(lin_vel) > 0.0 else 0.0
