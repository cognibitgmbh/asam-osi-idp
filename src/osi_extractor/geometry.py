from dataclasses import dataclass
import math
from typing import Union

import numpy as np
from osi3.osi_common_pb2 import Orientation3d, Vector3d


def euclidean_distance(vec1: Vector3d, vec2: Vector3d, ignore_z: bool = True) -> float:
    if ignore_z:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2)
    else:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2 + (vec1.z-vec2.z)**2)


def osi_vector_to_ndarray(vec: Vector3d) -> np.ndarray:
    return np.array([vec.x, vec.y, vec.z])

def angle_of_segment(line: np.ndarray, segment_id: int) -> float:
    vector = line[segment_id + 1] - line[segment_id]
    return np.arctan2(vector[1], vector[0])


@dataclass
class ProjectionResult:
    projected_point: np.ndarray
    segment_index: int
    segment_progress: float


def project_onto_line_segments(
    p: np.ndarray,
    seg_1: np.ndarray,
    seg_2: np.ndarray
) -> tuple[Union[float, np.ndarray], np.ndarray]:
    """
    Project point p onto one or more line segments defined by seg_1 and seg_2.

    Arguments:
    p      -- Point to project. Should be an ndarray of shape (3,)
    seg_1  -- Starting points for each line segment.
              Should be an ndarray of shape (n, 3)
    seg_2  -- End points for each line segement.
              Must be an ndarray of the same shape as seg_1

    Returns: 2 ndarrays
    - t of shape (n,)
    - v of shape (n, 3)
    For each i between 0 and 3: seg_1[i] + t[i] * v[i]
    is the projected point for the ith line segment.
    """
    v = seg_2 - seg_1
    t = np.einsum("ij,ij->i", v, p - seg_1) / np.einsum("ij,ij->i", v, v)
    return t, v


def closest_projected_point(
    p: np.ndarray,
    segment_points: np.ndarray,
) -> ProjectionResult:
    start = segment_points[:-1, :]
    end = segment_points[1:, :]
    t, v = project_onto_line_segments(p, start, end)
    relevant_indexes,  = np.nonzero((0 <= t) & (t <= 1))
    if len(relevant_indexes) <= 0:
        # determine the closest segment start
        d = start - p
        distance_squared = np.einsum("ij,ij->i", d, d)
        min_i = np.argmin(distance_squared)
        # also consider the final segment end
        last_i = end.shape[0] - 1
        last_d = end[last_i, :] - p
        if np.dot(last_d, last_d) < distance_squared[min_i]:
            return ProjectionResult(
                projected_point=end[last_i],
                segment_index=last_i,
                segment_progress=1.0
            )
        else:
            return ProjectionResult(
                projected_point=start[min_i],
                segment_index=min_i,
                segment_progress=0.0
            )
    t_relevant = t[relevant_indexes]
    projected = (start[relevant_indexes, :]
                 + np.expand_dims(t_relevant, axis=-1) * v[relevant_indexes, :])
    d = projected - p
    distance_squared = np.einsum("ij,ij->i", d, d)
    i = np.argmin(distance_squared)
    return ProjectionResult(
        projected_point=projected[i],
        segment_index=relevant_indexes[i],
        segment_progress=t_relevant[i],
    )

class Orientation:
    _matrix: np.ndarray

    @staticmethod
    def from_osi(osi_obj: Orientation3d) -> 'Orientation':
        return Orientation(yaw=osi_obj.yaw, pitch=osi_obj.pitch, roll=osi_obj.roll)

    def __init__(self, yaw: float, pitch: float, roll: float):
        # matrix construction is based on:
        # https://opensimulationinterface.github.io/osi-documentation/#_coordinate_systems_and_reference_points
        sin_yaw = np.sin(yaw)
        sin_pitch = np.sin(pitch)
        sin_roll = np.sin(roll)
        cos_yaw = np.cos(yaw)
        cos_pitch = np.cos(pitch)
        cos_roll = np.cos(roll)
        self._matrix = np.array([
            [cos_pitch*cos_yaw, cos_pitch*sin_yaw, -sin_pitch],
            [sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw, sin_roll*sin_pitch*sin_yaw + cos_roll*cos_yaw, sin_roll*cos_pitch],
            [cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw, cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw, cos_roll*cos_pitch],
        ])

    def rotate_vector(self, vector: np.ndarray) -> np.ndarray:
        return self._matrix @ vector


def angle_between_vectors(vec1: np.ndarray, vec2: np.ndarray) -> float:
    cosine =  np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
    return float(np.arccos(cosine))



##########################################################################
# TODO From cb_utils, organize libraries! Put back into cognibot repo
def get_distance_to_reference_point(
    vehicle_location , heading: float, reference_location
) -> float:
    v_heading = np.array([math.cos(heading), math.sin(heading), 0.0])
    v_orthogonal = np.array(
        [
            reference_location[0] - vehicle_location[0],
            reference_location[1] - vehicle_location[1],
            0.0,
        ]
    )

    absolute_distance = norm(v_orthogonal)

    cross_product = np.cross(v_heading, v_orthogonal)

    if cross_product[2] < 0:
        signed_distance = absolute_distance
    else:
        signed_distance = absolute_distance * (-1.0)

    return signed_distance

def norm(x):
    if x.size == 2:
        norm = np.sqrt(x[0] ** 2 + x[1] ** 2)
    elif x.size == 3:
        norm = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
    else:
        raise ValueError("Wrong dimensions for norm calculation.")

    return norm