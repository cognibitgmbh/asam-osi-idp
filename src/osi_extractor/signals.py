from collections.abc import Mapping
from types import MappingProxyType
from typing import Optional

import math
import numpy as np
from osi3.osi_groundtruth_pb2 import GroundTruth

from .geometry import Orientation, ProjectionResult, angle_between_vectors, osi_vector_to_ndarray
from .lanegraph import LaneGraphNode, LaneGraph
from .road import RoadManager, RoadSignal

SIGN_VIEW_NORMAL = np.array([1, 0, 0])
SIGN_MAX_ANGLE = math.pi / 4.0  # 45 degrees
MAX_SIGN_DISTANCE = 10.0


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

    def assign_signs_to_roads(self, gt: GroundTruth):
        for osi_sign in gt.traffic_sign:
            main_sign_base = osi_sign.main_sign.base
            position = osi_vector_to_ndarray(main_sign_base.position)
            orientation = Orientation.from_osi(main_sign_base.orientation)
            lane = self._find_closest_lane(position, orientation, max_distance=MAX_SIGN_DISTANCE)
            if lane is None:
                print(f'WARNING: Could not assign traffic sign {osi_sign.id.value} to a lane')
                continue
            road = self.road_manager.get_road(lane)
            if road is None:
                print(f'WARNING: Could not assign traffic sign {osi_sign.id.value} to a road')
                continue
            road_signal = RoadSignal(
                road_id=road.road_id,
                road_s=road.object_road_s(lane, position),
                closest_lane=lane,
                osi_signal=osi_sign,
            )
            road.signals.append(road_signal)

    def _find_closest_lane(self, position: np.ndarray, orientation: Orientation, max_distance: float = float('inf')) \
            -> Optional[LaneGraphNode]:
        closest_lane = None
        closest_distance = max_distance
        for lane in self.lane_graph.iterate_nodes():
            projection = lane.data.project_onto_centerline(position)
            if not lane_check_sign_orientation(lane, orientation, projection):
                continue
            distance = np.linalg.norm(position - projection.projected_point)
            if distance < closest_distance:
                closest_lane = lane
                closest_distance = distance
        return closest_lane
