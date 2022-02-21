import sys
import threading
import math

from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_lane_pb2 import Lane
from osi3.osi_common_pb2 import Vector3d
from typing import Dict


from  osi_iterator import OSI3GroundTruthIterator, UDPGroundTruthIterator

def euclidean_distance(vec1: Vector3d, vec2: Vector3d, ignore_z: bool = True) -> float:
    if ignore_z:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2)
    else:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2 + (vec1.z-vec2.z)**2)

def find_lane_pice_for_coord(lane: Lane, coordinate: Vector3d) -> int:
    id_closest_lane_piece = -1
    distance_closest_lane_piece = float("Inf")
    centerline = lane.classification.centerline
    for i, xyz in enumerate(centerline):
        cur_distance = euclidean_distance(coordinate, xyz)
        if cur_distance < distance_closest_lane_piece:
            distance_closest_lane_piece = cur_distance
            id_closest_lane_piece = i
    if id_closest_lane_piece == 0:
        a = id_closest_lane_piece
        b = id_closest_lane_piece + 1
    elif id_closest_lane_piece == len(centerline) -1:
        a = id_closest_lane_piece -1
        b = id_closest_lane_piece
    elif (euclidean_distance(centerline[id_closest_lane_piece-1], centerline[id_closest_lane_piece]) -
            euclidean_distance(centerline[id_closest_lane_piece-1], coordinate)
            >  
            euclidean_distance(centerline[id_closest_lane_piece+1], centerline[id_closest_lane_piece]) 
            - euclidean_distance(centerline[id_closest_lane_piece+1], coordinate)):
        a = id_closest_lane_piece -1
        b = id_closest_lane_piece
    else:
        a = id_closest_lane_piece
        b = id_closest_lane_piece +1
    return a

class OSI3Extractor:
    lanes: Dict[int, Lane] = {}
    def __init__(self, ip_addr: str, port: int = 48198):
        self.ground_truth_iterator = UDPGroundTruthIterator(ip_addr, port)
        self.thread = threading.Thread(target=self.thread_target)
    
    def start(self) -> threading.Thread:
        self.thread.start()
        return self.thread

    def thread_target(self):
        use_deprecated_assigned_lane = True
        for ground_truth in self.ground_truth_iterator:
            if len(ground_truth.lane) != 0:
                self.update_lanes(ground_truth.lane)
            for object in ground_truth.moving_object:
                if object.id == ground_truth.host_vehicle_id:
                    if use_deprecated_assigned_lane:
                        lane_id = object.assigned_lane_id[0].value
                    else:
                        lane_id = object.moving_object_classification.assigned_lane_id[0].value
                    print("Id of closest lane piece: " + str(find_lane_pice_for_coord(self.lanes[lane_id], object.base.position)))
                    break
            print("----------------------------------------------------")

    def update_lanes(self, new_lanes: Lane):
        for l in new_lanes:
            self.lanes[l.id.value] = l

            
def main():
    if len(sys.argv) != 2:
        print(f"Usage:\n{sys.argv[0]} <port>")
        sys.exit(1)
    osi_extractor = OSI3Extractor("127.0.0.1", int(sys.argv[1]))
    osi_extractor.start()


if __name__ == "__main__":
    main()
