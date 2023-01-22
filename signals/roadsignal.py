from osi3.osi_trafficsign_pb2 import TrafficSign

from dataclasses import dataclass

from ..lanegraph import LaneGraphNode


@dataclass(frozen=True)
class RoadSignal:
    road_id: int
    road_s: tuple[float,float]
    closest_lane: LaneGraphNode
    osi_signal: TrafficSign
