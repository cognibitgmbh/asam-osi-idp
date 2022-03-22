import sys
import threading
import math
import random
import time

from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_lane_pb2 import Lane
from osi3.osi_common_pb2 import Vector3d
from osi3.osi_object_pb2 import MovingObject
from typing import Dict, List, Optional

from osi_iterator import UDPGroundTruthIterator
from state import RoadState, MovingObjectState, StaticObstacle, State

use_deprecated_assigned_lane = True

def get_assigned_lane_id(moving_obj: MovingObject) -> Optional[int]:
    # TODO: What happens with additional assigned lanes?
    assigned_lane_id = moving_obj.moving_object_classification.assigned_lane_id
    if use_deprecated_assigned_lane:
        assigned_lane_id = moving_obj.assigned_lane_id
    if len(assigned_lane_id) == 0:
        return None
    return assigned_lane_id[0].value

def get_all_assigned_lane_ids(moving_obj: MovingObject) -> Optional[int]:
    assigned_lane_id = moving_obj.moving_object_classification.assigned_lane_id
    if use_deprecated_assigned_lane:
        assigned_lane_id = moving_obj.assigned_lane_id
    return [lane.value for lane in assigned_lane_id]

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

def euclidean_distance(vec1: Vector3d, vec2: Vector3d, ignore_z: bool = True) -> float:
    if ignore_z:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2)
    else:
        return math.sqrt((vec1.x-vec2.x)**2 + (vec1.y-vec2.y)**2 + (vec1.z-vec2.z)**2)

def calculate_piece_progress(point:Vector3d, start: Vector3d, end: Vector3d) -> float:
    #https://stackoverflow.com/questions/61341712/calculate-projected-point-location-x-y-on-given-line-startx-y-endx-y

    l2 = euclidean_distance(start, end, ignore_z = False) ** 2
    if l2 == 0:
      raise Exception('a and b are the same points')
      
    t = ((point.x - start.x)*(end.x - start.x)  + (point.y - start.y)*(end.y - start.y)  + (point.z - start.z)*(end.z - start.z)) / l2
    t = max(0, min(1, t))
    
    #projection = Vector3d(x=a.x + t*(b.x -a.x), y=a.y + t*(b.y -a.y), z=a.z + t*(b.z -a.z))
    return t 

def find_lane_piece_for_coord(lane: Lane, coordinate: Vector3d, return_progress: bool = False) -> int:
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
    if return_progress:
        return a, calculate_piece_progress(coordinate, centerline[a], centerline[b])
    return a


def convert_to_moving_object_state(object: MovingObject): #TODO: needs lanes as parameter
    m_o_s = MovingObjectState()
    m_o_s.simulator_id = object.id.value
    m_o_s.object_type = object.type 
    m_o_s.dimensions = object.base.dimension
    m_o_s.location = object.base.position
    m_o_s.velocity = object.base.velocity
    m_o_s.acceleration = object.base.acceleration
    m_o_s.yaw_angle = object.base.orientation.yaw
    m_o_s.pitch_angle = object.base.orientation.pitch
    m_o_s.roll_angle = object.base.orientation.roll
    m_o_s.lane_ids = get_all_assigned_lane_ids(object)
    light_state = object.vehicle_classification.light_state
    m_o_s.indicator_signal = light_state.indicator_state
    m_o_s.brake_light = light_state.brake_light_state
    m_o_s.front_fog_light = light_state.front_fog_light
    m_o_s.rear_fog_light = light_state.rear_fog_light
    m_o_s.head_light = light_state.head_light
    m_o_s.high_beam = light_state.high_beam
    m_o_s.reversing_light = light_state.reversing_light
    m_o_s.license_plate_illumination_rear = light_state.license_plate_illumination_rear
    m_o_s.emergency_vehicle_illumination = light_state.emergency_vehicle_illumination
    m_o_s.service_vehicle_illumination = light_state.service_vehicle_illumination

    #TODO: Replace 'None' with actual values
    m_o_s.heading_angle = None
    m_o_s.lane_position = None
    m_o_s.road_id = None
    m_o_s.road_s = None
    return m_o_s



class OSI3Extractor:
    host_vehicle_id: Optional[int] = None
    host_vehicle: Optional[MovingObject] = None
    lanes: Dict[int, Lane] = {}
    lane_curvatures: Dict[int, List[float]] = {}
    current_state: State = None
    
    def __init__(self, ip_addr: str, port: int = 48198):
        self.ground_truth_iterator = UDPGroundTruthIterator(ip_addr, port)
        self.thread = threading.Thread(target=self.thread_target)
    
    def start(self) -> threading.Thread:
        self.thread.start()
        return self.thread

    def thread_target(self):
        arising_state: State = None
        for ground_truth in self.ground_truth_iterator:
            if len(ground_truth.lane) != 0:
                self.update_lanes(ground_truth.lane)
            self.host_vehicle_id = ground_truth.host_vehicle_id
            m_o_list: list[MovingObjectState] = []
            for i, object in enumerate(ground_truth.moving_object):
                if object.id.value != i:
                    print("Waring: the objects position in the moving_object list does not equal object.id")
                m_o_list.append(convert_to_moving_object_state(object)) 

#                if object.id == self.host_vehicle_id:
#                    self.host_vehicle = object
#                    lane_id = get_assigned_lane_id(object)
#                    closest_lane_piece = find_lane_piece_for_coord(self.lanes[lane_id], object.base.position)
           #         print("Id of closest lane piece: " + str(closest_lane_piece))
           #         print("curvature of closest piece: " + str(self.lane_curvatures[lane_id][closest_lane_piece]))
#                    break
        #    print("----------------------------------------------------")
            print("How many moving objects: " + str(len(m_o_list)))                
            s_o_list: list[StaticObstacle] = []
            for object in ground_truth.stationary_object:
                s_o = StaticObstacle(object.base.dimension, object.base.position) # TODO: Create Testcase
                s_o_list.append(s_o)
            print("How many stationary objects: " + str(len(s_o_list)))                
            self.current_state = State(m_o_list, s_o_list, self.host_vehicle_id)

    def _get_ego_lane_id(self) -> int:
        if self.host_vehicle is None:
            raise RuntimeError("No host vehicle")
        lane_id = get_assigned_lane_id(self.host_vehicle)
        if lane_id is None:
            raise RuntimeError("Host vehicle has no assigned lane")
        return lane_id


    def update_lanes(self, new_lanes: Lane):
        for l in new_lanes:
            self.lanes[l.id.value] = l
            self.lane_curvatures[l.id.value] = calc_curvature_for_lane(l)

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

    # TODO: lane type of neighbouring lanes
    def get_ego_lane_type(self) -> tuple[int, int]:
        classification = self.lanes[self._get_ego_lane_id()].classification
        return classification.type, classification.subtype

    def currently_on_intersection(self) -> bool:
        classification = self.lanes[self._get_ego_lane_id()].classification
        # TODO: somehow deal with this "magic constant"
        return classification.type == 4

    def get_road_z(self) -> float:
        self.host_vehicle.base.position
        ego_lane = self.lanes[self._get_ego_lane_id()]
        piece_id, t = find_lane_piece_for_coord(ego_lane , self.host_vehicle.base.position, return_progress = True)
        ego_centerline = ego_lane.classification.centerline
        return t*ego_centerline[piece_id].z + (1-t)*ego_centerline[piece_id + 1].z


def main():
    if len(sys.argv) != 2:
        print(f"Usage:\n{sys.argv[0]} <port>")
        sys.exit(1)
    osi_extractor = OSI3Extractor("127.0.0.1", int(sys.argv[1]))
    osi_extractor.start()
    for i in range(100):
        try:
            time.sleep(1)
            print("Current road curvature: " + str(osi_extractor.get_road_curvature()))
            print("Current road curvature change: " + str(osi_extractor.get_road_curvature_change()))
            lane_type, lane_subtype = osi_extractor.get_ego_lane_type()
            print(f"Current lane type: {lane_type}, {lane_subtype}")
            road_z = osi_extractor.get_road_z()
            print(f"Current road z: {road_z}")
        except RuntimeError as e:
            print(e)


if __name__ == "__main__":
    main()
