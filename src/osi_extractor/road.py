from dataclasses import dataclass
from typing import Optional

import numpy as np
import osi3.osi_lane_pb2 as lane_pb2

from osi3.osi_trafficsign_pb2 import TrafficSign
from .lane import LaneSubtype

from .lanegraph import LaneGraph, LaneGraphNode

SUBTYPE_ENTRY = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_ENTRY"].number
SUBTYPE_EXIT = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_EXIT"].number
SUBTYPE_NORMAL = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_NORMAL"].number
SUBTYPE_ONRAMP = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_ONRAMP"].number
SUBTYPE_OFFRAMP = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name[
    "SUBTYPE_OFFRAMP"].number
SUBTYPE_CONNECTINGRAMP = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name[
    "SUBTYPE_CONNECTINGRAMP"].number


@dataclass(frozen=True)
class RoadSignal:
    road_id: int
    road_s: tuple[float, float]
    closest_lane: LaneGraphNode
    osi_signal: TrafficSign


class Road:
    road_id: int
    _rightmost_lanes: list[LaneGraphNode]
    _rightmost_lanes_lengths: np.ndarray
    _total_distance: float
    on_highway: bool
    signals: list[RoadSignal]

    def __init__(self):
        self.signals = []

    def _get_rightmost_roadlane(self, lane: LaneGraphNode) -> LaneGraphNode:
        current_lane = lane
        while current_lane not in self._rightmost_lanes:
            current_lane = current_lane.right
            if current_lane is None:
                raise Exception(
                    f"Lane with id:{lane.id} seems to be not part of road with id: {self.road_id}")
        return current_lane

    def object_road_s(self, lane: LaneGraphNode, position: np.ndarray) -> tuple[float, float]:
        rightmost_lane: LaneGraphNode = self._get_rightmost_roadlane(lane)
        index = self._rightmost_lanes.index(rightmost_lane)
        projection = rightmost_lane.data.project_onto_centerline(position)
        distance = np.sum(self._rightmost_lanes_lengths[:index + 1]).item()
        distance -= rightmost_lane.data.distance_to_end(projection)
        return distance, self._total_distance

    def calculate_s_of_exit_end(self, exit_lane: LaneGraphNode) -> Optional[float]:
        if exit_lane.data.lane_subtype != LaneSubtype.EXIT:
            return None
        if exit_lane not in self._rightmost_lanes:
            return None
        index = self._rightmost_lanes.index(exit_lane)
        result = np.sum(self._rightmost_lanes_lengths[:index]).item()
        current_lane = exit_lane
        while current_lane.data.lane_subtype == LaneSubtype.EXIT:
            result += current_lane.data.centerline_total_distance
            index += 1
            if index >= len(self._rightmost_lanes):
                break
            current_lane = self._rightmost_lanes[index]
        return result


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
                    next_new_level_lane = self._same_road_right_neighbor(
                        current_new_level_lane)
                    if next_new_level_lane is None:
                        return current_new_level_lane
                    current_new_level_lane = next_new_level_lane
            current_old_level_lane = self._same_road_left_neighbor(
                current_old_level_lane)
        return None

    def _add_all_parallel_lanes_to_road(self, rightmost_lane: LaneGraphNode, road: Road):
        lane = rightmost_lane
        while lane is not None:
            if lane.id in self.lane_id_to_road_map:
                return
            self.lane_id_to_road_map[lane.id] = road
            lane = self._same_road_left_neighbor(lane)

    def _create_new_road_starting_with(self, lane: LaneGraphNode, road_id: int):
        new_road = Road()
        new_road.on_highway = False
        new_road.road_id = road_id
        new_road._rightmost_lanes = []
        new_road._total_distance = 0.0
        current_lane = lane
        while current_lane is not None:
            if current_lane.data.lane_subtype in (LaneSubtype.ENTRY, LaneSubtype.EXIT):
                new_road.on_highway = True
            self._add_all_parallel_lanes_to_road(current_lane, new_road)
            new_road._rightmost_lanes.append(current_lane)
            current_lane = self._get_next_mostright_lane(current_lane)
        temp_length = []
        for lane in new_road._rightmost_lanes:
            temp_length.append(lane.data.centerline_total_distance)
        new_road._rightmost_lanes_lengths = np.array(temp_length)
        new_road._total_distance = np.sum(new_road._rightmost_lanes_lengths)

    def _same_road_right_neighbor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]:
        road_independent_neighbor = lane.right
        if road_independent_neighbor is None:
            return None
        elif road_independent_neighbor.data.osi_lane.classification.type != lane.data.osi_lane.classification.type:
            return None
        elif road_independent_neighbor.id in self.lane_id_to_road_map:
            return None
        return road_independent_neighbor

    def _same_road_left_neighbor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]:
        road_independent_neighbor = lane.left
        if road_independent_neighbor is None:
            return None
        elif road_independent_neighbor.data.osi_lane.classification.type != lane.data.osi_lane.classification.type:
            return None
        elif road_independent_neighbor.id in self.lane_id_to_road_map and lane.id not in self.lane_id_to_road_map:
            raise Exception("This should not happen, since roads are created from right to left")
            #return None
        return road_independent_neighbor

    @staticmethod
    def _are_successing_lanes_same_road(predecessor, successor) -> bool:
        successor_subtype = successor.subtype
        predecessor_subtype = predecessor.subtype
        if predecessor.type != successor.type:
            return False
        if successor_subtype == predecessor_subtype:
            return True
        # TODO: maybe also allow NORMAL -> ONRAMP?
        if predecessor_subtype == LaneSubtype.NORMAL.value and successor_subtype in (
                LaneSubtype.NORMAL.value, LaneSubtype.EXIT.value):
            return True
        if predecessor_subtype == LaneSubtype.EXIT.value and successor_subtype == LaneSubtype.EXIT.value:
            return True
        if predecessor_subtype == LaneSubtype.ENTRY.value and successor_subtype in (
                LaneSubtype.NORMAL.value, LaneSubtype.ENTRY.value, LaneSubtype.EXIT.value):
            return True
        if predecessor_subtype == LaneSubtype.ONRAMP.value and successor_subtype == LaneSubtype.ONRAMP.value:
            return True
        if predecessor_subtype == LaneSubtype.OFFRAMP.value and successor_subtype in (
                LaneSubtype.ONRAMP.value, LaneSubtype.ONRAMP.value):
            return True
        if predecessor_subtype == LaneSubtype.CONNECTINGRAMP.value \
                and successor_subtype == LaneSubtype.CONNECTINGRAMP.value:
            return True
        return False

    def _same_road_successor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]:
        road_independent_successor = lane.successor
        if road_independent_successor is None:
            return None
        if road_independent_successor.id in self.lane_id_to_road_map:
            return None
        if self._are_successing_lanes_same_road(lane.data.osi_lane.classification,
                                                road_independent_successor.data.osi_lane.classification):
            return road_independent_successor
        else:
            return None

    def _same_road_predecessor(self, lane: LaneGraphNode) -> Optional[LaneGraphNode]:
        road_independent_predecessor = lane.predecessor
        if road_independent_predecessor is None:
            return None
        elif road_independent_predecessor.id in self.lane_id_to_road_map:
            return None
        elif self._are_successing_lanes_same_road(road_independent_predecessor.data.osi_lane.classification,
                                                lane.data.osi_lane.classification):
            return road_independent_predecessor
        else:
            return None

    def _is_rightmost_beginning_lane(self, lane: LaneGraphNode) -> bool:
        if self._same_road_right_neighbor(lane) is not None:
            return False
        current_lane = lane
        while current_lane is not None:
            if self._same_road_predecessor(current_lane) is not None:
                return False
            current_lane = self._same_road_left_neighbor(current_lane)
        return True

    def _create_roads(self, lane_graph: LaneGraph):
        last_next_road_id = -1
        next_road_id = 0
        while next_road_id != last_next_road_id:
            last_next_road_id = next_road_id
            for lane_id, lane in lane_graph._nodes.items():
                if lane.id in self.lane_id_to_road_map:
                    continue
                if self._is_rightmost_beginning_lane(lane):
                    self._create_new_road_starting_with(lane, next_road_id)
                    next_road_id += 1


    def get_road(self, lane: LaneGraphNode) -> Optional[Road]:
        if lane.id in self.lane_id_to_road_map:
            return self.lane_id_to_road_map[lane.id]
        else:
            return None
