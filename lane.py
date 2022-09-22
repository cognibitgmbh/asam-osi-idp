from __future__ import annotations
from dataclasses import dataclass

from typing import Iterable, Sequence

import numpy as np
from osi3.osi_common_pb2 import Vector3d
from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_lane_pb2 import Lane, LaneBoundary
import osi3.osi_lane_pb2 as lane_pb2

from curvature import Curvature
from geometry import (ProjectionResult, closest_projected_point,
                      osi_vector_to_ndarray)


OSI_LANE_TYPE_DRIVING = lane_pb2._LANE_CLASSIFICATION_TYPE.values_by_name["TYPE_DRIVING"].number
OSI_LANE_TYPE_INTERSECTION = lane_pb2._LANE_CLASSIFICATION_TYPE.values_by_name["TYPE_INTERSECTION"].number
OSI_LANE_TYPES_FOR_DRIVING = set((OSI_LANE_TYPE_DRIVING, OSI_LANE_TYPE_INTERSECTION))

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
        self.curvature = Curvature(self.centerline_matrix, self.centerline_distances)

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
        self.centerline_total_distance = np.sum(self.centerline_distances)

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

    def boundary_points_for_position(
        self,
        position: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        left = closest_projected_point(
            position, self.left_boundary_matrix).projected_point
        right = closest_projected_point(
            position, self.right_boundary_matrix).projected_point
        return left, right

    def project_onto_centerline(self, position: np.ndarray) -> ProjectionResult:
        return closest_projected_point(position, self.centerline_matrix)

    def distance_to_end(self, proj_res: ProjectionResult) -> float:
        distance = (self.centerline_distances[proj_res.segment_index]
                    * (1 - proj_res.segment_progress))
        distance += np.sum(
            self.centerline_distances[proj_res.segment_index+1:],
        )
        return distance

    def allows_for_driving(self) -> bool:
        return self.osi_lane.classification.type in OSI_LANE_TYPES_FOR_DRIVING

    def start_point(self) -> np.array:
        return self.centerline_matrix[0, :]

    def end_point(self) -> np.array:
        return self.centerline_matrix[-1, :]
