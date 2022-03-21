from typing import Union

import numpy as np
from osi3.osi_common_pb2 import Vector3d


def osi_vector_to_ndarray(vec: Vector3d) -> np.ndarray:
    return np.array([vec.x, vec.y, vec.z])


def project_onto_line_segment(
    p: np.ndarray,
    seg_1: np.ndarray,
    seg_2: np.ndarray
) -> Union[float, np.ndarray]:
    """
    Project point p onto one or more line segments defined by seg_1 and seg_2.

    Arguments:
    p      -- Point to project. Should be an ndarray of shape (3,)
    seg_1  -- Starting points for each line segment.
              Should be an ndarray of shape (n, 3)
    seg_2  -- End points for each line segement.
              Must be an ndarray of the same shape as seg_1

    Returns: ndarray t of shape (n,).
    For each i between 0 and 3: seg_1[i] + t[i] * (seg_2 - seg_1)[i]
    is the ith coordinate of the projected point.
    """
    v = seg_2 - seg_1
    t = np.einsum("ij,ij->i", v, p - seg_1) / np.einsum("ij,ij->i", v, v)
    return t


def main():
    p = np.array([0, 0, 0])
    s1 = np.array([[0, -1, -1], [1, -1, 2]])
    s2 = np.array([[1, 0, 1], [1, 3, 2]])
    t = project_onto_line_segment(p, s1, s2)
    print(t)


if __name__ == "__main__":
    main()
