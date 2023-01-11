import sys
import threading
import time
from os import environ

from osi3.osi_groundtruth_pb2 import GroundTruth

from lane import LaneData
from lanegraph import LaneGraph
from osi_iterator import UDPGroundTruthIterator
from output.driver_update import DriverUpdate
from output.esmini_output_sender import EsminiOutputSender
from output.osi_trace_output_sender import OsiTraceOutputSender
from output.raw_update import RawUpdate
from road import RoadManager
from state import State
from state_builder import create_state


class OSI3Extractor:
    def __init__(self, ip_addr: str, port: int = 48198, ego_id: int = 0):
        self.ground_truth_iterator = UDPGroundTruthIterator(ip_addr, port)
        self.ego_id = ego_id
        self._stop_requested = False
        self.thread = threading.Thread(target=self._thread_target)
        self.lane_data: dict[int, LaneData] = {}
        self.lane_graph = LaneGraph(self.lane_data)
        self.road_manager = RoadManager(self.lane_graph)
        self._current_state: State = None
        if environ.get('OUTPUT_FILE') is not None:
            self.output = OsiTraceOutputSender(environ['OUTPUT_FILE'])
        elif environ.get('OUTPUT_PORT') is not None and environ['OUTPUT_ADDRESS'] is not None:
            self.output = EsminiOutputSender(environ['OUTPUT_ADDRESS'],
                                             int(environ['OUTPUT_PORT']))
        else:
            self.output = None

    def __enter__(self):
        self.ground_truth_iterator.open()
        if self.output is not None:
            self.output.open()
        self.thread.start()
        return self

    def __exit__(self, exc_type, *args) -> bool:
        self.ground_truth_iterator.close()
        print("pre join")
        self.thread.join()
        print("post join")
        if self.output is not None:
            self.output.close()
        return exc_type is None

    def _thread_target(self):
        for ground_truth in self.ground_truth_iterator:
            if len(ground_truth.lane) != 0:
                self.update_lane_data(ground_truth)
            self.host_vehicle_id = ground_truth.host_vehicle_id.value
            self._current_state = create_state(ground_truth, self.lane_graph,
                                               self.road_manager, self.ego_id)
            vehicle = self._current_state.moving_objects[0]
            print(f"Ego lane: {vehicle.lane_ids[0]}", end=", ")

    def current_state(self) -> State:
        return self._current_state

    def update_lane_data(self, gt: GroundTruth):
        for lane in gt.lane:
            id: int = lane.id.value
            self.lane_data[id] = LaneData(gt, lane)
        self.lane_graph = LaneGraph(self.lane_data)
        self.road_manager = RoadManager(self.lane_graph)

    def send_raw_update(self, raw_update: RawUpdate):
        if self.output is None:
            raise RuntimeError("No output sender defined") 
        self.output.send_raw_update(raw_update)

    def send_driver_update(self, driver_update: DriverUpdate):
        if self.output is None:
            raise RuntimeError("No output sender defined") 
        self.output.send_driver_update(driver_update)


def main():
    if len(sys.argv) != 4:
        print(f"Usage:\n{sys.argv[0]} <listen ip> <port> <ego vehicle id>")
        sys.exit(1)
    osi_extractor = OSI3Extractor(ip_addr=sys.argv[1],
                                  port=int(sys.argv[2]),
                                  ego_id=int(sys.argv[3]))
    with osi_extractor:
        for _ in range(100):
            try:
                time.sleep(1)
            except RuntimeError as e:
                print(e)


if __name__ == "__main__":
    main()
