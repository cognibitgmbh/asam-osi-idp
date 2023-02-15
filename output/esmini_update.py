from dataclasses import dataclass


@dataclass
class DriverInputUpdate:
    id: int
    steer: float
    brake: float
    throttle: float

    def __init__(self, id: int, steer: float, brake: float, throttle: float):
        self.id = id
        self.steer = steer
        self.brake = brake
        self.throttle = throttle


@dataclass
class XYHSpeedSteeringUpdate:
    # see '6.6.8. UDPDriverController' in https://github.com/esmini/esmini/blob/master/docs/user_guide.adoc
    id: int
    x: float
    y: float
    h: float
    speed: float
    steering_wheel_angle: float
    dead_reckon: bool # esmini will move entity according to latest received heading and speed (basically extrapolating)

    def __init__(self,
                 id: int,
                 x: float,
                 y: float,
                 h: float,
                 speed: float = 0.0,
                 steering_wheel_angle: float = 0.0,
                 dead_reckon: bool = False):
        self.id = id
        self.x = x
        self.y = y
        self.h = h
        self.speed = speed
        self.steering_wheel_angle = steering_wheel_angle
        self.dead_reckon = dead_reckon
