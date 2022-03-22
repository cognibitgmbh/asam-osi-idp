import math
from typing import Union

import numpy as np
from osi3.osi_common_pb2 import Vector3d


def euclidean_distance(vec1: Vector3d, vec2: Vector3d, ignore_z: bool = True) -> float:
    if ignore_z:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2)
    else:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2 + (vec1.z-vec2.z)**2)


def osi_vector_to_ndarray(vec: Vector3d) -> np.ndarray:
    return np.array([vec.x, vec.y, vec.z])


def project_onto_line_segment(
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
    seg_1: np.ndarray,
    seg_2: np.ndarray,
) -> tuple[np.ndarray, Union[float, np.ndarray]]:
    t, v = project_onto_line_segment(p, seg_1, seg_2)
    mask = (0 <= t) & (t <= 1)
    t_masked = t[mask]
    projected = seg_1[mask, :] + t_masked * v[mask, :]
    w = projected - p
    distance_squared = np.einsum("ij,ij->i", w, w)
    i = np.argmin(distance_squared)
    return projected[i], t_masked[i]


def main():
    p = np.array([0, 0, 0])
    s1 = np.array([[0, -1, -1], [1, -1, 2]])
    s2 = np.array([[1, 0, 1], [1, 3, 2]])
    t = project_onto_line_segment(p, s1, s2)
    print(t)


if __name__ == "__main__":
    main()
