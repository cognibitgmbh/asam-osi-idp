import time

import math
from osi_extractor import State
from osi_extractor.output.esmini_update import DriverInputUpdate


class DriverController:
    vehicle_id: int

    def __init__(self, vehicle_id: int):
        self.vehicle_id = vehicle_id

    def process_next_state(self, state: State) -> DriverInputUpdate:
        ego_state = state.moving_objects[self.vehicle_id]
        speed = ego_state.velocity.total_kph
        print(f"Current speed: {speed} km/h")
        speed_limit = ego_state.road_state.speed_limit
        if speed_limit is None or speed < speed_limit:
            return DriverInputUpdate(id=self.vehicle_id,
                                     steer=0.0, brake=0.0, throttle=1.0)
        else:
            return DriverInputUpdate(id=self.vehicle_id,
                                     steer=0.0, brake=1.0, throttle=0.0)
