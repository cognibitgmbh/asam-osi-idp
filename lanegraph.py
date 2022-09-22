from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np

from lane import LaneData


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


class LaneGraph:
    def __init__(self, lane_dict: dict[int, LaneData]):
        self._nodes = {id: LaneGraphNode(id=id, data=data) for id, data
                       in lane_dict.items() if data.allows_for_driving()}
        for id, lane in lane_dict.items():
            if lane.allows_for_driving():
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
