from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from osi3.osi_common_pb2 import Dimension3d, Vector3d, Orientation3d
from osi3.osi_object_pb2 import MovingObject
from osi3.osi_trafficlight_pb2 import TrafficLight

import osi_extractor.speedlimit_logic as speedlimit_logic
from .deprecated_handler import get_all_assigned_lane_ids
from .geometry import angle_of_segment, osi_vector_to_ndarray
from .lane import LaneBoundaryMarkingType, LaneSubtype, LaneType
from .lanegraph import LaneGraph, NeighboringLaneSignal
from .road import Road, RoadManager, RoadSignal

YAW_IS_ALREADY_RELATIVE = True  # TODO: decide on where to set such flags


@dataclass(init=False)
class RoadState:
    curvature: float
    curvature_change: float
    lane_width: float
    lane_position: float
    distance_to_lane_end: NeighboringLaneSignal[float]
    distance_to_ramp: NeighboringLaneSignal[float]
    distance_to_next_exit: Optional[float]
    lane_type: NeighboringLaneSignal[tuple[LaneType, LaneSubtype]]
    left_lane_marking: LaneBoundaryMarkingType
    right_lane_marking: LaneBoundaryMarkingType
    road_z: float
    road_angle: float
    relative_object_heading_angle: float
    # road_topography: ?
    road_on_highway: bool
    road_on_junction: bool
    same_road_as_ego: bool
    speed_limit: Optional[int] = None  # Or more info?
    traffic_signs: list[RoadSignal] = None  # Based on sensor?
    traffic_lights: list[TrafficLight] = None # Based on sensor?

    # if ego_road_id is None, the state object will assume that this moving object is the ego vehicle
    def __init__(self, lane_graph: LaneGraph, mos: MovingObjectState, road: Road, ego_road_id: 'int | None'):
        current_position = osi_vector_to_ndarray(mos.location)
        current_lane_id = mos.lane_ids[0]
        current_lane_data = lane_graph.get_lane_data(current_lane_id)
        if current_lane_data is None:
            raise RuntimeError(f"Moving object {mos.simulator_id} is on lane"
                               f" {current_lane_id} which is not meant for driving")
        centerline_projection = current_lane_data.project_onto_centerline(
            current_position,
        )
        self.curvature = current_lane_data.curvature.get_road_curvature(
            centerline_projection.segment_index, centerline_projection.segment_progress)
        self.curvature_change = current_lane_data.curvature.get_road_curvature_change(
            centerline_projection.segment_index
        )
        self.distance_to_lane_end = lane_graph.distance_to_lane_end(
            current_lane_id,
            current_position,
        )
        self.distance_to_next_exit = lane_graph.distance_to_next_exit(
            current_lane_id,
            current_position,
        )
        self.distance_to_ramp = lane_graph.distance_to_ramp(
            current_lane_id,
            current_position,
        )
        lane_boundary_left, lane_boundary_right = (
            current_lane_data.boundary_points_for_position(current_position)
        )
        self.lane_width = np.linalg.norm(lane_boundary_left - lane_boundary_right)
        self.lane_position = (
            np.linalg.norm(current_position - lane_boundary_left) / self.lane_width
        )
        self.lane_type = lane_graph.neighbor_lane_types(current_lane_id)
        self.left_lane_marking = current_lane_data.get_lane_boundary_marking_for_position(current_position, left=True)
        self.right_lane_marking = current_lane_data.get_lane_boundary_marking_for_position(current_position, left=False)
        self.road_on_junction = current_lane_data.lane_type == LaneType.INTERSECTION
        _, _, self.road_z = centerline_projection.projected_point
        self.road_angle = angle_of_segment(
            current_lane_data.centerline_matrix, centerline_projection.segment_index)
        if YAW_IS_ALREADY_RELATIVE:
            self.relative_object_heading_angle = mos.orientation.yaw
        else:
            self.relative_object_heading_angle = (
                mos.orientation.yaw - self.road_angle + np.pi) % (2*np.pi) - np.pi
        self.road_on_highway = road.on_highway
        if ego_road_id is None:
            self.same_road_as_ego = True
        else:
            self.same_road_as_ego = road.road_id == ego_road_id
        ignore_exit_speed_signs = self.road_on_highway and self.lane_type.current_lane[1] != LaneSubtype.EXIT
        self.speed_limit = speedlimit_logic.calculate_speedlimit(road, mos.road_s, ignore_exit_speed_signs)
        self.traffic_signs = road.signals
        self.traffic_lights = None # TODO should remove this


@dataclass(init=False)
class MovingObjectState:
    simulator_id: int
    object_type: int
    dimensions: Dimension3d  # TODO: is called dimension without s in osi
    location: Vector3d  # TODO: is called position in osi
    velocity: Vector3d
    acceleration: Vector3d
    orientation: Orientation3d
    lane_ids: list[int]
    road_id: Optional[int]
    road_s: Optional[tuple[float, float]]
    road_state: Optional[RoadState]
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

    # if ego_road_id is None, the state object will assume that this moving object is the ego vehicle
    def __init__(self, mo: MovingObject, lane_graph: LaneGraph, road_manager: RoadManager, ego_road_id: 'int | None'):
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

        if len(self.lane_ids) == 0 or lane_graph._nodes.get(self.lane_ids[0]) == None:
            self.road_id = None
            self.road_s = None
            self.road_state = None
            return

        lane_graph_node = lane_graph._nodes[self.lane_ids[0]]
        road_of_lane = road_manager.get_road(lane_graph_node)
        if road_of_lane is None:
            self.road_id = None
            self.road_s = None
            self.road_state = None
            return
        self.road_id = road_of_lane.road_id
        if lane_graph_node.data.centerline_matrix.shape[0] <= 0:
            self.road_s = None
            self.road_state = None
            return
        self.road_s = road_of_lane.object_road_s(
            lane_graph_node, osi_vector_to_ndarray(self.location))
        self.road_state = RoadState(lane_graph, self, road_of_lane, ego_road_id)
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
