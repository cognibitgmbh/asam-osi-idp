import sys
import threading
import math
import time

from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_lane_pb2 import Lane
from osi3.osi_common_pb2 import Vector3d
from osi3.osi_object_pb2 import MovingObject
from typing import Dict, List, Optional

from osi_iterator import UDPGroundTruthIterator
from state import RoadState, MovingObjectState, StationaryObstacle, State
from state_builder import create_state
from deprecated_handler import get_assigned_lane_id, get_all_assigned_lane_ids
from curvature import calc_curvature_for_lane


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
            self.current_state = create_state(ground_truth)
            print("How many moving objects: " + str(len(self.current_state.moving_objects)))
            print("How many stationary objects: " + str(len(self.current_state.stationary_obstacles)))

    def get_current_state(self) -> State:
        return self.current_state

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

    # TODO: lane type of neighbouring lanes
    def get_ego_lane_type(self) -> tuple[int, int]:
        classification = self.lanes[self._get_ego_lane_id()].classification
        return classification.type, classification.subtype

    def currently_on_intersection(self) -> bool:
        classification = self.lanes[self._get_ego_lane_id()].classification
        # TODO: somehow deal with this "magic constant"
        return classification.type == 4


def main():
    if len(sys.argv) != 2:
        print(f"Usage:\n{sys.argv[0]} <port>")
        sys.exit(1)
    osi_extractor = OSI3Extractor("127.0.0.1", int(sys.argv[1]))
    osi_extractor.start()
    for i in range(100):
        try:
            time.sleep(1)
        except RuntimeError as e:
            print(e)


if __name__ == "__main__":
    main()
