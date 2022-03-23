from __future__ import annotations

from dataclasses import dataclass

from osi3.osi_common_pb2 import Dimension3d, Vector3d, Orientation3d
from osi3.osi_object_pb2 import MovingObject
from osi3.osi_trafficlight_pb2 import TrafficLight
from osi3.osi_trafficsign_pb2 import TrafficSign

from deprecated_handler import get_all_assigned_lane_ids
from lane import LaneData


@dataclass(init=False)
class RoadState:
    curvature: float
    curvature_change: float
    lane_width: float
    distance_to_lane_end: float
    distance_to_ramp: float
    distance_to_next_exit: float
    lane_markings: list[int]
    lane_type: tuple[int, int]
    speed_limit: int  # Or more info?
    traffic_signs: list[TrafficSign]  # Based on sensor?
    traffic_lights: list[TrafficLight]  # Based on sensor?
    road_z: float
    # road_topography: ?
    road_on_highway: bool
    road_on_junction: bool
    road_in_main_direction: bool

    def __init__(self, lane_data: dict[int, LaneData], mos: MovingObjectState):
        ego_lane_data = lane_data[mos.lane_ids[0]]
        self.lane_width = ego_lane_data.width_at_position(mos.location)
        osi_lane_classification = (
            ego_lane_data.osi_lane.classification
        )
        self.lane_type = (
            osi_lane_classification.type, osi_lane_classification.subtype
        )
        # TODO: somehow deal with this "magic constant"
        self.road_on_highway = self.lane_type[0] == 4
        # TODO: initialize everything else


@dataclass(init=False)
class MovingObjectState:
    simulator_id: int
    object_type: int
    dimensions: Dimension3d  # TODO: is called dimension without s in osi
    location: Vector3d  # TODO: is called position in osi
    velocity: Vector3d
    acceleration: Vector3d
    orientation: Orientation3d
    heading_angle: float
    lane_ids: list[int]
    lane_position: float
    road_id: int
    road_s: float
    indicator_signal: int
    brake_light: int
    front_fog_light: int
    rear_fog_light: int
    head_light: int
    high_beam: int
    reversing_light: int
    license_plate_illumination_rear: int
    emergency_vehicle_illumination: int
    service_vehicle_illumination: int
    road_state: RoadState

    def __init__(self, mo: MovingObject):
        self.simulator_id = mo.id.value
        self.object_type = mo.type
        self.dimensions = mo.base.dimension
        self.location = mo.base.position
        self.velocity = mo.base.velocity
        self.acceleration = mo.base.acceleration
        self.orientation = mo.base.orientation
        self.lane_ids = get_all_assigned_lane_ids(mo)
        light_state = mo.vehicle_classification.light_state
        self.indicator_signal = light_state.indicator_state
        self.brake_light = light_state.brake_light_state
        self.front_fog_light = light_state.front_fog_light
        self.rear_fog_light = light_state.rear_fog_light
        self.head_light = light_state.head_light
        self.high_beam = light_state.high_beam
        self.reversing_light = light_state.reversing_light
        self.license_plate_illumination_rear = light_state.license_plate_illumination_rear
        self.emergency_vehicle_illumination = light_state.emergency_vehicle_illumination
        self.service_vehicle_illumination = light_state.service_vehicle_illumination
        # TODO: Replace 'None' with actual values
        self.heading_angle = None
        self.lane_position = None
        self.road_id = None
        self.road_s = None


@dataclass
class StationaryObstacle:
    dimensions: Dimension3d
    location: Vector3d


@dataclass
class State:
    moving_objects: list[MovingObjectState]
    stationary_obstacles: list[StationaryObstacle]
    host_vehicle_id: int
