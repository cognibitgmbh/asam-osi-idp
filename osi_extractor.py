import sys
import threading
import time
from typing import Iterable

from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_lane_pb2 import Lane

from curvature import calc_curvature_for_lane
from lane import LaneData
from osi_iterator import UDPGroundTruthIterator
from state import State
from state_builder import create_state


class OSI3Extractor:
    def __init__(self, ip_addr: str, port: int = 48198):
        self.ground_truth_iterator = UDPGroundTruthIterator(ip_addr, port)
        self.thread = threading.Thread(target=self._thread_target)
        self.lane_data: dict[int, LaneData] = {}
        self.lane_curvatures: dict[int, list[float]] = {}
        self._current_state: State = None

    def start(self) -> threading.Thread:
        self.thread.start()
        return self.thread

    def _thread_target(self):
        arising_state: State = None
        for ground_truth in self.ground_truth_iterator:
            if len(ground_truth.lane) != 0:
                self.update_lane_data(ground_truth)
            self.host_vehicle_id = ground_truth.host_vehicle_id
            self.current_state = create_state(ground_truth)
            print("How many moving objects: " + str(len(self.current_state.moving_objects)))
            print("How many stationary objects: " + str(len(self.current_state.stationary_obstacles)))

    def current_state(self) -> State:
        return self._current_state

    def update_lane_data(self, gt: GroundTruth):
        for lane in gt.lane:
            id: int = lane.id.value
            self.lane_data[id] = LaneData(gt, lane)


def main():
    if len(sys.argv) != 3:
        print(f"Usage:\n{sys.argv[0]} <listen ip> <port>")
        sys.exit(1)
    osi_extractor = OSI3Extractor(sys.argv[1], int(sys.argv[2]))
    osi_extractor.start()
    for _ in range(100):
        try:
            time.sleep(1)
        except RuntimeError as e:
            print(e)


if __name__ == "__main__":
    main()
