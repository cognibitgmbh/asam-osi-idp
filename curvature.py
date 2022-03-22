from typing import List
from osi3.osi_lane_pb2 import Lane
from geometry import euclidean_distance
import math

def get_road_curvature(self):
    lane_id = self._get_ego_lane_id()
    piece_id, percentage = find_lane_piece_for_coord(self.lanes[lane_id], self.host_vehicle.base.position, return_progress=True) 
    return self.lane_curvatures[lane_id][piece_id]*(1-percentage) + self.lane_curvatures[lane_id][piece_id+1]*percentage

def get_road_curvature_change(self):
    lane_id = self._get_ego_lane_id()
    piece_id = find_lane_piece_for_coord(self.lanes[lane_id], self.host_vehicle.base.position) 

    curvature_difference = self.lane_curvatures[lane_id][piece_id + 1] - self.lane_curvatures[lane_id][piece_id]
    piece_length = euclidean_distance(self.lanes[lane_id].classification.centerline[piece_id], 
    self.lanes[lane_id].classification.centerline[piece_id + 1])
    return curvature_difference / piece_length

def calc_curvature_for_lane(lane: Lane) -> List[float]:
    centerline = lane.classification.centerline
    curvatures = []
    for i in range(len(centerline)):
        if i == 0 or i == len(centerline) - 1:
            curvatures.append(0.0)
        else:
            p1 = centerline[i-1]
            p2 = centerline[i]
            p3 = centerline[i+1]

            a = euclidean_distance(p1, p2)
            b = euclidean_distance(p2, p3)
            c = euclidean_distance(p3, p1)
            try:
                A = 1/4 * math.sqrt(4*a*a*b*b - (a*a + b*b -c*c)**2) # https://en.wikipedia.org/wiki/Heron%27s_formula
            except ValueError:
                A = 0.0
            curvatures.append(4*A/(a*b*c)) # https://en.wikipedia.org/wiki/Menger_curvature
    return curvatures

