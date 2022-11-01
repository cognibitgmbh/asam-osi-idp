from dataclasses import dataclass
from typing import Generic, Iterable, Optional, TypeVar

import numpy as np

from lane import LaneData, LaneSubtype


SUCCESSOR_MAX_DISTANCE = 0.1


@dataclass
class LaneGraphNode:
    id: int
    data: LaneData
    right: Optional['LaneGraphNode'] = None
    left: Optional['LaneGraphNode'] = None
    predecessor: Optional['LaneGraphNode'] = None
    successor: Optional['LaneGraphNode'] = None

    def __repr__(self) -> str:
        right = self.right.id if self.right is not None else None
        left = self.left.id if self.left is not None else None
        pre = self.predecessor.id if self.predecessor is not None else None
        succ = self.successor.id if self.successor is not None else None
        return (f"LaneGraphNode(id={self.id}, right={right}, left={left}"
                              f", predecessor={pre}, successor={succ})")


class MultipleNeighborsError(Exception):
    def __init__(self, lane_id: int, side: str):
        super().__init__()
        self.lane_id = lane_id
        self.side = side

    def __str__(self) -> str:
        return f"OSI lane {self.lane_id} has multiple {self.side} neighbors"


T = TypeVar("T")


@dataclass
class NeighboringLaneSignal(Generic[T]):
    current_lane: T
    left_lane: Optional[T] = None
    right_lane: Optional[T] = None


class LaneGraph:
    def __init__(self, lane_dict: dict[int, LaneData]):
        self._nodes = {id: LaneGraphNode(id=id, data=data) for id, data
                       in lane_dict.items() if data.lane_type.allows_for_driving()}
        for id, lane in lane_dict.items():
            if lane.lane_type.allows_for_driving():
                self._add_lane_neighbours(id)
        self._compute_successors()

    def __str__(self) -> str:
        return str(self._nodes)

    def _add_lane_neighbours(self, id: int):
        node = self._nodes[id]
        classification = node.data.osi_lane.classification
        left_ids: Iterable[int] = (id.value for id
                                   in classification.left_adjacent_lane_id)
        for left_id in left_ids:
            if left_id not in self._nodes:
                continue
            left_node = self._nodes[left_id]
            if node.left is None:
                node.left = left_node
            elif node.left.id != left_id:
                raise MultipleNeighborsError(id, "left")
            if left_node.right is None:
                left_node.right = node
            elif left_node.right.id != id:
                raise MultipleNeighborsError(left_id, "right")
        right_ids: Iterable[int] = (id.value for id
                                    in classification.right_adjacent_lane_id)
        for right_id in right_ids:
            if right_id not in self._nodes:
                continue
            right_node = self._nodes[right_id]
            if node.right is None:
                node.right = right_node
            elif node.right.id != right_id:
                raise MultipleNeighborsError(id, "right")
            if right_node.left is None:
                right_node.left = node
            elif right_node.left.id != id:
                raise MultipleNeighborsError(right_id, "left")

    def _compute_successors(self):
        for id, node in self._nodes.items():
            end = node.data.end_point()
            for other_id, other_node in self._nodes.items():
                if id == other_id:
                    continue
                start = other_node.data.start_point()
                if np.linalg.norm(start - end) <= SUCCESSOR_MAX_DISTANCE:
                    self._add_successor(pre=node, succ=other_node)

    def _add_successor(self, pre: LaneGraphNode, succ: LaneGraphNode):
        if pre.successor is not None:
            raise MultipleNeighborsError(pre.id, "successor")
        if succ.predecessor is not None:
            raise MultipleNeighborsError(succ.id, "predecessor")
        pre.successor = succ
        succ.predecessor = pre

    def get_lane_data(self, id: int) -> Optional[LaneData]:
        if id not in self._nodes:
            return None
        return self._nodes[id].data

    def _distance_to_lane_end(self, node: LaneGraphNode, position: np.ndarray) -> float:
        projection = node.data.project_onto_centerline(position)
        distance = node.data.distance_to_end(projection)
        while node.successor is not None:
            node = node.successor
            distance += node.data.centerline_total_distance
        return distance

    def distance_to_lane_end(self, lane_id: int, position: np.ndarray) -> NeighboringLaneSignal[float]:
        node = self._nodes[lane_id]
        result = NeighboringLaneSignal(
            current_lane=self._distance_to_lane_end(node, position),
        )
        if node.left is not None:
            result.left_lane = self._distance_to_lane_end(node.left, position)
        if node.right is not None:
            result.right_lane = self._distance_to_lane_end(node.right, position)
        return result

    def _get_rightmost_lane(self, node: LaneGraphNode) -> LaneGraphNode: 
        while node.right is not None:
            node = node.right
        return node
    
    def _next_lane_node(self, current_node: LaneGraphNode) -> Optional[LaneGraphNode]:
        while current_node is not None:
            if current_node.successor is not None:
                return self._get_rightmost_lane(current_node.successor)
            current_node = current_node.left
        return None
    
    def distance_to_next_exit(self, lane_id: int, position: np.ndarray) -> Optional[float]:
        node = self._nodes[lane_id]
        projection = node.data.project_onto_centerline(position)
        distance = node.data.distance_to_end(projection)
        node = self._get_rightmost_lane(node)
        while not node.data.lane_subtype == LaneSubtype.EXIT:
            node = self._next_lane_node(node)
            if node is None:
                return None
            distance += node.data.centerline_total_distance
        return distance
        

