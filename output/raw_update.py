from dataclasses import dataclass
from typing import Optional

from osi3.osi_common_pb2 import Vector3d, Orientation3d


@dataclass
class RawUpdate:
    id: int
    vehicle_type: int
    position: Vector3d
    orientation: Orientation3d

    indicator_state: Optional[int]
    brake_light_state: Optional[int]
    front_fog_light: Optional[int]
    rear_fog_light: Optional[int]
    head_light: Optional[int]
    high_beam: Optional[int]
    reversing_light: Optional[int]
    license_plate_illumination_rear: Optional[int]
    emergency_vehicle_illumination: Optional[int]
    service_vehicle_illumination: Optional[int]

    def __init__(self,
                id: int,
                 position: Vector3d,
                 orientation: Orientation3d,
                 indicator_state: Optional[int] = None,
                 brake_light_state: Optional[int] = None,
                 front_fog_light: Optional[int] = None,
                 rear_fog_light: Optional[int] = None,
                 head_light: Optional[int] = None,
                 high_beam: Optional[int] = None,
                 reversing_light: Optional[int] = None,
                 license_plate_illumination_rear: Optional[int] = None,
                 emergency_vehicle_illumination: Optional[int] = None,
                 service_vehicle_illumination: Optional[int] = None):

        self.id = id
        self.position = position
        self.orientation = orientation
        self.indicator_state = indicator_state
        self.brake_light_state = brake_light_state
        self.front_fog_light = front_fog_light
        self.rear_fog_light = rear_fog_light
        self.head_light = head_light
        self.high_beam = high_beam
        self.reversing_light = reversing_light
        self.license_plate_illumination_rear = license_plate_illumination_rear
        self.emergency_vehicle_illumination = emergency_vehicle_illumination
        self.service_vehicle_illumination = service_vehicle_illumination
