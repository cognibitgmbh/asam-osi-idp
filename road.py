from hashlib import new
from locale import currency
from typing import Optional

import numpy as np
import osi3.osi_lane_pb2 as lane_pb2

from lanegraph import LaneGraph, LaneGraphNode

SUBTYPE_ENTRY = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_ENTRY"].number
SUBTYPE_EXIT = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_EXIT"].number
SUBTYPE_NORMAL = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_NORMAL"].number
SUBTYPE_ONRAMP = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_ONRAMP"].number
SUBTYPE_OFFRAMP = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_OFFRAMP"].number
SUBTYPE_CONNECTINGRAMP = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_CONNECTINGRAMP"].number

class Road:
    road_id: int
    _rightmost_lanes: list[LaneGraphNode]  
    _rightmost_lanes_lengths: np.ndarray
    _total_distance: float

    def _get_rightmost_roadlane(self, lane: LaneGraphNode) -> LaneGraphNode:
        current_lane = lane
        while(current_lane not in self._rightmost_lanes):
            current_lane = current_lane.right
            if current_lane is None:
                raise Exception(f"Lane with id:{lane.id} seems to be not part of road with id: {self.road_id}")
        return current_lane

    def object_road_s(self, lane: LaneGraphNode, position: np.ndarray) -> float:
        rightmost_lane: LaneGraphNode = self._get_rightmost_roadlane(lane)
        index = self._rightmost_lanes.index(rightmost_lane)
        projection = rightmost_lane.data.project_onto_centerline(position)
        distance = rightmost_lane.data.distance_to_end(projection)
        distance += np.sum(self._rightmost_lanes_lengths[index + 1:]).item()
        return distance / self._total_distance


class RoadManager:
    lane_id_to_road_map: dict[int, Road] = {} 

    def __init__(self, lane_graph: LaneGraph) -> None:
        self._create_roads(lane_graph)
    
    def _get_next_mostright_lane(self, old_level_lane: LaneGraphNode) -> Optional[LaneGraphNode]:
        current_old_level_lane = old_level_lane
        while current_old_level_lane is not None:
            successor = self._same_road_successor(current_old_level_lane)
            if successor is not None:
                current_new_level_lane = successor
                while True:
                    next_new_level_lane = self._same_road_right_neighbor(current_new_level_lane)
                    if next_new_level_lane is None:
                        return current_new_level_lane
                    current_new_level_lane = next_new_level_lane
            current_old_level_lane = self._same_road_left_neighbor(current_old_level_lane)
        return None

    def _add_all_parallel_lanes_to_road(self, rightmost_lane: LaneGraphNode, road: Road):
        lane = rightmost_lane
        while lane is not None:
            self.lane_id_to_road_map[lane.id] = road
            lane = self._same_road_left_neighbor(lane)
            

    def _create_new_road_starting_with(self, lane: LaneGraphNode, road_id: int):
        new_road = Road()
        new_road.road_id = road_id
        new_road._rightmost_lanes = []
        new_road._total_distance = 0.0
        current_lane = lane
        while current_lane is not None:
            self._add_all_parallel_lanes_to_road(current_lane, new_road)
            new_road._rightmost_lanes.append(current_lane)
            current_lane = self._get_next_mostright_lane(current_lane)
        temp_length = []
        for lane in new_road._rightmost_lanes:
            temp_length.append(lane.data.centerline_total_distance)
        new_road._rightmost_lanes_lengths = np.array(temp_length)
        new_road._total_distance = np.sum(new_road._rightmost_lanes_lengths)

    
    def _same_road_right_neighbor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]: #TODO: Don't know if this is good
        road_independent_neighbor = lane.right
        if road_independent_neighbor is None:
            return None
        else:
            neighbor_subtype = road_independent_neighbor.data.osi_lane.classification.subtype
            lane_subtype = lane.data.osi_lane.classification.subtype
            if road_independent_neighbor.data.osi_lane.classification.type != lane.data.osi_lane.classification.type:
                return None
            return road_independent_neighbor

    def _same_road_left_neighbor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]: #TODO: Don't know if this is good
        road_independent_neighbor = lane.left
        if road_independent_neighbor is None:
            return None
        else:
            neighbor_subtype = road_independent_neighbor.data.osi_lane.classification.subtype
            lane_subtype = lane.data.osi_lane.classification.subtype
            if road_independent_neighbor.data.osi_lane.classification.type != lane.data.osi_lane.classification.type:
                return None
            return road_independent_neighbor

    def _same_road_successor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]: #TODO: Don't know if this is good
        road_independent_successor = lane.successor
        if road_independent_successor is None:
            return None
        else:
            successor_subtype = road_independent_successor.data.osi_lane.classification.subtype
            lane_subtype = lane.data.osi_lane.classification.subtype
            if road_independent_successor.data.osi_lane.classification.type != lane.data.osi_lane.classification.type:
                return None
            if successor_subtype == lane_subtype:
                return road_independent_successor
            
            if lane_subtype == SUBTYPE_EXIT and successor_subtype != SUBTYPE_EXIT:
                return None
            if lane_subtype != SUBTYPE_ENTRY and successor_subtype == SUBTYPE_ENTRY:
                return None
            return road_independent_successor

    def _same_road_precessor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]: #TODO: Don't know if this is good
        road_independent_precessor = lane.predecessor
        if road_independent_precessor is None:
            return None
        else:
            precessor_subtype = road_independent_precessor.data.osi_lane.classification.subtype
            lane_subtype = lane.data.osi_lane.classification.subtype
            if road_independent_precessor.data.osi_lane.classification.type != lane.data.osi_lane.classification.type:
                return None
            if precessor_subtype == lane_subtype:
                return road_independent_precessor
            
            if precessor_subtype == SUBTYPE_EXIT and lane_subtype != SUBTYPE_EXIT:
                return None
            if precessor_subtype != SUBTYPE_ENTRY and lane_subtype == SUBTYPE_ENTRY:
                return None
            return road_independent_precessor


    def _is_rightmost_beginning_lane(self, lane: LaneGraphNode) -> bool:
        if self._same_road_right_neighbor(lane) is not None:
            return False
        current_lane = lane
        while current_lane is not None:
            if self._same_road_precessor(current_lane) is not None:
                return False
            current_lane = self._same_road_left_neighbor(current_lane)
        return True


    def _create_roads(self, lane_graph: LaneGraph):
        next_road_id = 0
        for lane_id, lane in lane_graph._nodes.items():
            if lane.id in self.lane_id_to_road_map:
                continue
            if self._is_rightmost_beginning_lane(lane):
                self._create_new_road_starting_with(lane, next_road_id)
                next_road_id += 1




            
    def get_road_id(self, lane: LaneGraphNode) -> Optional[int]:
        return self.lane_id_to_road_map[lane.id].road_id
    def object_road_s(self, lane: LaneGraphNode, position: np.ndarray) -> float:
        return self.lane_id_to_road_map[lane.id].object_road_s(lane, position)
        
