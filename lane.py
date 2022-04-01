from __future__ import annotations
from dataclasses import dataclass

from typing import Iterable, Optional, Sequence

import numpy as np
from osi3.osi_common_pb2 import Vector3d
from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_lane_pb2 import Lane, LaneBoundary

from curvature import Curvature
from geometry import (ProjectionResult, closest_projected_point,
                      osi_vector_to_ndarray)


@dataclass
class NeighboringLaneInfo:
    lane_id: int
    projection_result: ProjectionResult


@dataclass
class NeighboringLanes:
    left_lane: Optional[NeighboringLaneInfo] = None
    right_lane: Optional[NeighboringLaneInfo] = None


def get_lane_boundary_from_ground_truth(
    gt: GroundTruth,
    boundary_id: int
) -> LaneBoundary:
    for boundary in gt.lane_boundary:
        if boundary.id.value == boundary_id:
            return boundary
    raise RuntimeError(f"Missing data for lane boundary {boundary_id}")


def boundaries_to_ndarray(
    boundaries: Iterable[LaneBoundary],
    total_points: int,
) -> np.ndarray:
    a = np.empty((total_points, 3))
    i: int = 0
    for boundary in boundaries:
        for bpoint in boundary.boundary_line:
            a[i, :] = osi_vector_to_ndarray(bpoint.position)
            i += 1
    return a


class LaneData:
    def __init__(self, gt: GroundTruth, osi_lane: Lane):
        self.osi_lane = osi_lane
        self._reverse_direction = not (
            self.osi_lane.classification.centerline_is_driving_direction
        )
        self._init_boundaries(gt)
        self._init_centerline()
        self.curvature = Curvature(
            self.centerline_matrix, self.centerline_distances
        )
        # neighbor attributes should be initialized on demand
        self.left_neighbor_points = None
        self.left_neighbor_ids = None
        self.right_neighbor_points = None
        self.right_neighbor_ids = None

    def _init_centerline(self):
        centerline: Sequence[Vector3d] = self.osi_lane.classification.centerline
        self.centerline_len = len(centerline)
        self.centerline_matrix = np.empty((self.centerline_len, 3))
        for i in range(self.centerline_len):
            osi_vector = (centerline[i] if not self._reverse_direction
                          else centerline[self.centerline_len - 1 - i])
            self.centerline_matrix[i, :] = osi_vector_to_ndarray(osi_vector)
        self.centerline_distances = np.linalg.norm(
            self.centerline_matrix[1:, :] - self.centerline_matrix[:-1, :],
            axis=1,
        )

    def _init_boundaries(self, gt: GroundTruth):
        left_boundaries = [
            get_lane_boundary_from_ground_truth(gt, id.value)
            for id in self.osi_lane.classification.left_lane_boundary_id
        ]
        right_boundaries = [
            get_lane_boundary_from_ground_truth(gt, id.value)
            for id in self.osi_lane.classification.right_lane_boundary_id
        ]
        if self._reverse_direction:
            left_boundaries, right_boundaries = (right_boundaries,
                                                 left_boundaries)
        n_left_points = sum(len(b.boundary_line) for b in left_boundaries)
        n_right_points = sum(len(b.boundary_line) for b in right_boundaries)
        self.left_boundary_matrix = boundaries_to_ndarray(left_boundaries,
                                                          n_left_points)
        self.right_boundary_matrix = boundaries_to_ndarray(right_boundaries,
                                                           n_right_points)

    # This is not called by __init__() as it requires all LaneData objects
    # to be initialized.
    # Currently, this function is only used by neighboring_lane_positions(),
    # which automatically calls this if necessary.
    def _init_neighboring_lanes(self, lane_data: dict[int, LaneData]):
        left_neighbors = [
            lane_data[id.value]
            for id in self.osi_lane.classification.left_adjacent_lane_id
        ]
        right_neighbors = [
            lane_data[id.value]
            for id in self.osi_lane.classification.right_adjacent_lane_id
        ]
        if self._reverse_direction:
            left_neighbors, right_neighbors = right_neighbors, left_neighbors
        n_left_points = sum(lane_data.centerline_len
                            for lane_data in left_neighbors)
        n_right_points = sum(lane_data.centerline_len
                             for lane_data in right_neighbors)
        self.left_neighbor_points = np.empty((n_left_points, 3))
        self.left_neighbor_ids = np.empty((n_left_points,))
        index: int = 0
        for left_lane in left_neighbors:
            next_index = index + left_lane.centerline_len
            self.left_neighbor_points[index:next_index, :] = (
                left_lane.centerline_matrix
            )
            self.left_neighbor_ids[index:next_index] = (
                left_lane.osi_lane.id.value
            )
            index = next_index
        self.right_neighbor_points = np.empty((n_right_points, 3))
        self.right_neighbor_ids = np.empty((n_right_points,))
        index = 0
        for right_lane in right_neighbors:
            next_index = index + right_lane.centerline_len
            self.right_neighbor_points[index:next_index, :] = (
                right_lane.centerline_matrix
            )
            self.right_neighbor_ids[index:next_index] = (
                right_lane.osi_lane.id.value
            )
            index = next_index

    def boundary_points_for_position(
        self,
        position: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        left = closest_projected_point(
            position, self.left_boundary_matrix).projected_point
        right = closest_projected_point(
            position, self.right_boundary_matrix).projected_point
        return left, right

    def _project_neighboring_lane(
        self,
        position: np.ndarray,
        neighbor_points: np.ndarray,
        neighbor_ids: np.ndarray,
    ) -> Optional[NeighboringLaneInfo]:
        if neighbor_points.shape[0] < 1:
            return None
        projection = closest_projected_point(position, neighbor_points)
        neighbor_id = neighbor_ids[projection.segment_index]
        if neighbor_id != neighbor_ids[projection.segment_index + 1]:
            # projection ended up between two different neighboring lanes
            return None
        distance = np.linalg.norm(projection.projected_point - position)
        p1, p2 = self.boundary_points_for_position(position)
        lane_width = np.linalg.norm(p2 - p1)
        if distance > 4 * lane_width:
            return None
        min_neighbor_index = np.nonzero(neighbor_ids == neighbor_id)[0][0]
        projection.segment_index -= min_neighbor_index
        return NeighboringLaneInfo(lane_id=neighbor_id,
                                   projection_result=projection)

    def neighboring_lanes_for_position(
        self,
        position: np.ndarray,
        lane_data: dict[int, LaneData]
    ) -> NeighboringLanes:
        if self.left_neighbor_ids is None:
            self._init_neighboring_lanes(lane_data)
        return NeighboringLanes(
            left_lane=self._project_neighboring_lane(
                position, self.left_neighbor_points, self.left_neighbor_ids
            ),
            right_lane=self._project_neighboring_lane(
                position, self.right_neighbor_points, self.right_neighbor_ids
            ),
        )

    def project_onto_centerline(self, position: np.ndarray) -> ProjectionResult:
        return closest_projected_point(position, self.centerline_matrix)

    def distance_to_end(self, proj_res: ProjectionResult) -> float:
        if proj_res.segment_index >= len(self.centerline_distances):
            return 0.0
        distance = (self.centerline_distances[proj_res.segment_index]
                    * (1 - proj_res.segment_progress))
        distance += np.sum(
            self.centerline_distances[proj_res.segment_index+1:],
        )
        return distance
