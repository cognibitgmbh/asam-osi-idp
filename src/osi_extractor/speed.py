import math
from osi3.osi_common_pb2 import Vector3d

class Speed:
    _velocity_x_mps: float
    _velocity_y_mps: float
    _velocity_z_mps: float

    def __init__(self, x_mps: float, y_mps: float, z_mps: float):
        self._velocity_x_mps = x_mps
        self._velocity_y_mps = y_mps
        self._velocity_z_mps - z_mps

    @staticmethod
    def from_osi_velocity(velocity: Vector3d) -> 'Speed':
        return Speed(velocity.x, velocity.y, velocity.z)

    @property
    def x_mps(self):
        return self._velocity_x_mps

    @property
    def y_mps(self):
        return self._velocity_y_mps

    @property
    def z_mps(self):
        return self._velocity_z_mps

    @property
    def total_mps(self):
        return math.sqrt(self._velocity_x_mps ** 2 + self._velocity_y_mps ** 2 + self._velocity_z_mps ** 2)

    @property
    def x_kph(self):
        return 3.6 * self._velocity_x_mps

    @property
    def y_kph(self):
        return 3.6 * self._velocity_y_mps

    @property
    def z_kph(self):
        return 3.6 * self._velocity_z_mps

    @property
    def total_kph(self):
        return 3.6 * math.sqrt(self._velocity_x_mps ** 2 + self._velocity_y_mps ** 2 + self._velocity_z_mps ** 2)
