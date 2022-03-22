from dataclasses import dataclass

from osi3.osi_common_pb2 import Dimension3d, Vector3d
from osi3.osi_trafficlight_pb2 import TrafficLight
from osi3.osi_trafficsign_pb2 import TrafficSign


@dataclass
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


@dataclass
class MovingObjectState:
    def __init__(self):
        pass #TODO: make it possible to create object without all arguments
    simulator_id: int
    object_type: int
    dimensions: Dimension3d #TODO: is called dimension without s in osi
    location: Vector3d #TODO: is called position in osi
    velocity: Vector3d
    acceleration: Vector3d
    yaw_angle: float
    pitch_angle: float
    roll_angle: float
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


@dataclass
class StationaryObstacle:
    dimensions: Dimension3d
    location: Vector3d


@dataclass
class State:
    moving_objects: list[MovingObjectState]
    stationary_obstacles: list[StationaryObstacle]
    host_vehicle_id: int
