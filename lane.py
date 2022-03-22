import numpy as np
from osi3.osi_lane_pb2 import Lane, LaneBoundary
from osi3.osi_common_pb2 import Vector3d
from osi3.osi_groundtruth_pb2 import GroundTruth

from geometry import osi_vector_to_ndarray, closest_projected_point


def get_lane_boundary_from_ground_truth(
    gt: GroundTruth,
    boundary_id: int
) -> LaneBoundary:
    for boundary in gt.lane_boundary:
        if boundary.id.value == boundary_id:
            return boundary
    raise RuntimeError(f"Missing data for lane boundary {boundary_id}")


class LaneData:
    def __init__(self, gt: GroundTruth, osi_lane: Lane):
        self.osi_lane = osi_lane
        left_boundaries = [
            LaneBoundaryState(gt, id.value)
            for id in self.osi_lane.classification.left_lane_boundary_id
        ]
        right_boundaries = [
            LaneBoundaryState(gt, id.value)
            for id in self.osi_lane.classification.right_lane_boundary_id
        ]
        n_left_segments = sum(len(b.segments) for b in left_boundaries)
        n_right_segments = sum(len(b.segments) for b in right_boundaries)
        self.left_start_points = np.empty((n_left_segments, 3))
        self.left_end_points = np.empty((n_left_segments, 3))
        i: int = 0
        for left_boundary in left_boundaries:
            for start, end in left_boundary.segments:
                self.left_start_points[i, :] = start
                self.left_end_points[i, :] = end
                i += 1
        self.right_start_points = np.empty((n_right_segments, 3))
        self.right_end_points = np.empty((n_right_segments, 3))
        i = 0
        for right_boundary in right_boundaries:
            for start, end in right_boundary.segments:
                self.right_start_points[i, :] = start
                self.right_end_points[i, :] = end
                i += 1

    def width_at_position(self, position: Vector3d) -> float:
        p = osi_vector_to_ndarray(position)
        left, _ = closest_projected_point(
            p, self.left_start_points, self.left_end_points,
        )
        right, _ = closest_projected_point(
            p, self.right_start_points, self.right_end_points,
        )
        return np.linalg.norm(right - left)


class LaneBoundaryState:
    def __init__(self, gt: GroundTruth, boundary_id: int):
        self.osi_boundary = get_lane_boundary_from_ground_truth(
            gt, boundary_id,
        )
        self.segments: list[tuple[np.ndarray, np.ndarray]] = []
        line = self.osi_boundary.boundary_line
        for i in range(len(line) - 1):
            self.segments.append((
                osi_vector_to_ndarray(line[i].position),
                osi_vector_to_ndarray(line[i + 1].position)
            ))
