import math
from collections.abc import Mapping
from types import MappingProxyType
from typing import Optional

import numpy as np
from osi3.osi_groundtruth_pb2 import GroundTruth

from .roadsignal import RoadSignal
from ..geometry import angle_between_vectors, euclidean_distance, Orientation, osi_vector_to_ndarray, ProjectionResult
from ..lanegraph import LaneGraph, LaneGraphNode
from ..road import RoadManager


SIGN_VIEW_NORMAL = np.array([1, 0, 0])
SIGN_MAX_ANGLE = math.pi / 4.0          # 45 degrees


def lane_check_sign_orientation(lane: LaneGraphNode, orientation: Orientation, projection: ProjectionResult) -> bool:
    lane_point1, lane_point2 = lane.data.segment_points(projection.segment_index)
    lane_reverse_direction = orientation.rotate_vector(lane_point1 - lane_point2)
    return angle_between_vectors(lane_reverse_direction, SIGN_VIEW_NORMAL) <= SIGN_MAX_ANGLE


class RoadAssignmentBuilder:
    lane_graph: LaneGraph
    road_manager: RoadManager

    def __init__(self, lane_graph: LaneGraph, road_manager: RoadManager):
        self.lane_graph = lane_graph
        self.road_manager = road_manager

    def build_assignment(self, gt: GroundTruth) -> Mapping[int, list[RoadSignal]]:
        assignment = {}
        for osi_sign in gt.traffic_sign:
            main_sign_base = osi_sign.main_sign.base
            position = osi_vector_to_ndarray(main_sign_base.position)
            orientation = Orientation.from_osi(main_sign_base.orientation)
            lane = self._find_closest_lane(position, orientation)
            if lane is None:
                print(f'WARNING: Could not assign traffic sign {osi_sign.id.value} to a lane')
                continue
            road = self.road_manager.get_road(lane)
            road_signal = RoadSignal(
                road_id=road.road_id,
                road_s=road.object_road_s(lane, position),
                closest_lane=lane,
                osi_signal=osi_sign,
            )
            if road.road_id not in assignment:
                assignment[road.road_id] = [road_signal]
            else:
                assignment[road.road_id].append(road_signal)
        return MappingProxyType(assignment)

    def _find_closest_lane(self, position: np.ndarray, orientation: Orientation) -> Optional[LaneGraphNode]:
        closest_lane = None
        closest_distance = float('inf')
        for lane in self.lane_graph.iterate_nodes():
            projection = lane.data.project_onto_centerline(position)
            if not lane_check_sign_orientation(lane, orientation, projection):
                continue
            distance = euclidean_distance(position, projection.projected_point, ignore_z=True)
            if closest_lane is None or distance < closest_distance:
                closest_lane = lane
                closest_distance = distance
        return closest_lane
