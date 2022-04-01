from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, Optional, TypeVar

import numpy as np
from osi3.osi_common_pb2 import Dimension3d, Orientation3d, Vector3d
from osi3.osi_object_pb2 import MovingObject
from osi3.osi_trafficlight_pb2 import TrafficLight
from osi3.osi_trafficsign_pb2 import TrafficSign

from deprecated_handler import get_all_assigned_lane_ids
from geometry import ProjectionResult, angle_of_segment, osi_vector_to_ndarray
from lane import LaneData, NeighboringLanes

T = TypeVar("T")


@dataclass
class NeighboringLaneSignal(Generic[T]):
    current_lane: T
    left_lane: Optional[T] = None
    right_lane: Optional[T] = None


YAW_IS_ALREADY_RELATIVE = True  # TODO: decide on where to set such flags


@dataclass(init=False)
class RoadState:
    curvature: float
    curvature_change: float
    lane_width: float
    lane_position: float
    distance_to_lane_end: NeighboringLaneSignal[float]
    distance_to_ramp: float
    distance_to_next_exit: float
    lane_markings: list[int]
    lane_type: tuple[int, int]
    speed_limit: int  # Or more info?
    traffic_signs: list[TrafficSign]  # Based on sensor?
    traffic_lights: list[TrafficLight]  # Based on sensor?
    road_z: float
    road_angle: float
    relative_object_heading_angle: float
    # road_topography: ?
    road_on_highway: bool
    road_on_junction: bool
    road_in_main_direction: bool

    def __init__(self, lane_data: dict[int, LaneData], mos: MovingObjectState):
        ego_position = osi_vector_to_ndarray(mos.location)
        # TODO: What happens if we have no assigned lane
        ego_lane_data = lane_data[mos.lane_ids[0]]
        centerline_projection = ego_lane_data.project_onto_centerline(
            ego_position,
        )
        self.curvature = ego_lane_data.curvature.get_road_curvature(
            centerline_projection.segment_index, centerline_projection.segment_progress)
        self.curvature_change = ego_lane_data.curvature.get_road_curvature_change(
            centerline_projection.segment_index
        )
        neighboring_lanes = ego_lane_data.neighboring_lanes_for_position(
            ego_position, lane_data,
        )
        self._init_distance_to_lane_end(
            lane=ego_lane_data,
            projection=centerline_projection,
            neighboring_lanes=neighboring_lanes,
            lane_dict=lane_data,
        )
        ego_lane_left, ego_lane_right = (
            ego_lane_data.boundary_points_for_position(ego_position)
        )
        self.lane_width = np.linalg.norm(ego_lane_left - ego_lane_right)
        self.lane_position = (
            np.linalg.norm(ego_position - ego_lane_left) / self.lane_width
        )
        osi_lane_classification = (
            ego_lane_data.osi_lane.classification
        )
        self.lane_type = (
            osi_lane_classification.type, osi_lane_classification.subtype
        )
        # TODO: somehow deal with this "magic constant"
        self.road_on_highway = self.lane_type[0] == 4
        _, _, self.road_z = centerline_projection.projected_point
        self.road_angle = angle_of_segment(
            ego_lane_data.centerline_matrix, centerline_projection.segment_index)
        if YAW_IS_ALREADY_RELATIVE:
            self.relative_object_heading_angle = mos.orientation.yaw
        else:
            self.relative_object_heading_angle = (
                mos.orientation.yaw - self.road_angle + np.pi) % (2*np.pi) - np.pi
        # TODO: initialize everything else

    def _init_distance_to_lane_end(
        self,
        lane: LaneData,
        projection: ProjectionResult,
        neighboring_lanes: NeighboringLanes,
        lane_dict: dict[int, LaneData],
    ):
        self.distance_to_lane_end = NeighboringLaneSignal(
            current_lane=lane.distance_to_end(projection),
        )
        if neighboring_lanes.left_lane is not None:
            left_data = lane_dict[neighboring_lanes.left_lane.lane_id]
            self.distance_to_lane_end.left_lane = left_data.distance_to_end(
                neighboring_lanes.left_lane.projection_result,
            )
        if neighboring_lanes.right_lane is not None:
            right_data = lane_dict[neighboring_lanes.right_lane.lane_id]
            self.distance_to_lane_end.right_lane = right_data.distance_to_end(
                neighboring_lanes.right_lane.projection_result,
            )


@dataclass(init=False)
class MovingObjectState:
    simulator_id: int
    object_type: int
    dimensions: Dimension3d  # TODO: is called dimension without s in osi
    location: Vector3d  # TODO: is called position in osi
    velocity: Vector3d
    acceleration: Vector3d
    orientation: Orientation3d
    relative_object_heading_angle: float
    heading_angle: float
    lane_ids: list[int]
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

    def __init__(self, mo: MovingObject, lane_data: dict[int, LaneData]):
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
        self.road_state = RoadState(lane_data, self)
        if YAW_IS_ALREADY_RELATIVE:
            self.orientation.yaw = (
                self.orientation.yaw + self.road_state.road_angle + 2*np.pi) % (2*np.pi)


@dataclass
class StationaryObstacle:
    dimensions: Dimension3d
    location: Vector3d


@dataclass
class State:
    moving_objects: list[MovingObjectState]
    stationary_obstacles: list[StationaryObstacle]
    host_vehicle_id: int
