from typing import List, Tuple
from osi3.osi_common_pb2 import Vector3d, Dimension3d
from osi3.osi_trafficsign_pb2 import TrafficSign
from osi3.osi_trafficlight_pb2 import TrafficLight

class RoadState:
    curvature: float
    curvature_change: float
    lane_width: float
    distance_to_lane_end: float
    distance_to_ramp: float
    distance_to_next_exit: float
    lane_markings: List[int]
    lane_type: Tuple[int,int]
    speed_limit: int # Or more info?
    traffic_signs: List[TrafficSign] # Based on sensor?
    traffic_lights: List[TrafficLight] # Based on sensor?
    road_z: float
#    road_topography: ?
    road_on_highway: bool
    road_on_junction: bool
    road_in_main_direction: bool

class MovingObject:
    simulator_id: int
    object_type: int 
    dimensions: Dimension3d
    location: Vector3d
    velocity: Vector3d
    acceleration: Vector3d
    yaw_angle: float
    pitch_angle: float
    roll_angle: float
    heading_angle: float
    lane_id: int
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
    brake_light_state: int
    license_plate_illumination_rear: int
    emergency_vehicle_illumination: int
    service_vehicle_illumination: int

class StaticObstacle:
    obstacle_dimensions: Dimension3d
    obstacle_location: Vector3d


class State:
    road_state: RoadState
    moving_objects: List[MovingObject]
    static_obstacles: List[StaticObstacle]

