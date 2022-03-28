from typing import List
from osi3.osi_lane_pb2 import Lane
from osi3.osi_common_pb2 import Vector3d
from geometry import euclidean_distance
import math
import numpy as np


class Curvatrure():
    def __init__(self, centerline: np.ndarray, distances: np.ndarray):
        #        self._curvature_list = np.array(self._calc_curvature_for_lane_slow(lane))
        self._curvature_list = self._calc_curvature_for_line(centerline)
#        if not np.allclose(self._curvature_change_list, self._calc_curvature_for_line(line)):
#            raise RuntimeError("Wrong curvatrure computation")
        self._curvature_change_list = self._calc_curvature_change_list(
            self._curvature_list, distances)

    def get_road_curvature(self, segment_index: int, segment_process: float):
        return self._curvature_list[segment_index]*(1-segment_process) + self._curvature_list[segment_index+1]*segment_process

    def get_road_curvature_change(self, segment_index: int):
        return self._curvature_change_list[segment_index]

    def _calc_curvature_change_list(self, curvatures: np.ndarray, distances: np.ndarray):
        curvature_differences = curvatures[1:] - curvatures[:-1]
        curvature_changes = curvature_differences / distances
        return curvature_changes

    def _calc_curvature_for_line(self, centerline: np.ndarray) -> np.ndarray:
        c_0 = centerline[:-2, :]
        c_1 = centerline[1:-1, :]
        c_2 = centerline[2:, :]
        a_np = np.linalg.norm(
            c_1 - c_0,
            axis=1,
        )
        b_np = np.linalg.norm(
            c_2 - c_1,
            axis=1,
        )
        c_np = np.linalg.norm(
            c_2 - c_0,
            axis=1,
        )
        a_squared = np.square(a_np)
        b_squared = np.square(b_np)
        # https://en.wikipedia.org/wiki/Heron%27s_formula
        A = 1/4 * np.sqrt(np.maximum(0, 4*a_squared*b_squared -
                          np.square(a_squared + b_squared - np.square(c_np))))

        ret_val = np.zeros((centerline.shape[0],))
        # https://en.wikipedia.org/wiki/Menger_curvature
        ret_val[1:-1] = 4*A/(a_np*b_np*c_np)
        return ret_val
