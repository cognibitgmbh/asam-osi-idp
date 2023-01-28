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

class SynchronOSI3Ectractor:
    def __init__(self, rec_ip_addr: str, rec_port: int = 48198, ego_id: int = 0, esmini_ip_addr: str = None, esmini_port: int = None):
        self.ground_truth_iterator = UDPGroundTruthIterator(rec_ip_addr, rec_port)
        self.ego_id = ego_id
        self.lane_data: dict[int, LaneData] = {}
        self.lane_graph = LaneGraph(self.lane_data)
        self.road_manager = RoadManager(self.lane_graph)
        if environ.get('OUTPUT_FILE') is not None:
            self.output = OsiTraceOutputSender(environ['OUTPUT_FILE'])
        elif esmini_ip_addr is not None and esmini_port is not None:
            self.output = EsminiOutputSender(esmini_ip_addr, esmini_port)
        else:
            self.output = None

    def __enter__(self):
        self.ground_truth_iterator.open()
        self._ground_truth_iter = self.ground_truth_iterator.__iter__()
        if self.output is not None:
            self.output.open()
        return self

    def __exit__(self, exc_type, *args) -> bool:
        self.ground_truth_iterator.close()
        if self.output is not None:
            self.output.close()
        return exc_type is None

    def get_next_state(self) -> State:
        ground_truth = self._ground_truth_iter.__next__()
        if len(ground_truth.lane) != 0:
            self.update_lane_data(ground_truth)
        self.host_vehicle_id = ground_truth.host_vehicle_id.value
        state = create_state(ground_truth, self.lane_graph,
                                           self.road_manager, self.ego_id)
        vehicle = state.moving_objects[0]
        print(f"Ego lane: {vehicle.lane_ids[0]}", end=", ")
        return state

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

    def send_empty_update(self, object_id: int):
        if self.output is None:
            raise RuntimeError("No output sender defined") 
        self.output.send_empty_update(object_id)


class AsynchronOSI3Extractor(SynchronOSI3Ectractor):

    def __init__(self, ip_addr: str, port: int = 48198, ego_id: int = 0, esmini_ip_addr: str = None, esmini_port: int = None):
        super(AsynchronOSI3Extractor, self).__init__(ip_addr, port, ego_id, esmini_ip_addr, esmini_port)
        self._current_state: State = None
        self.thread = threading.Thread(target=self._thread_target)

    def __enter__(self):
        super(AsynchronOSI3Extractor, self).__enter__()
        self.thread.start()
        return self

    def __exit__(self, exc_type, *args) -> bool:
        return_value = super(AsynchronOSI3Extractor, self).__exit__(exc_type, *args)
        self.thread.join()
        return return_value

    def _thread_target(self):
        while True:
            self._current_state = self.get_next_state()

    def current_state(self) -> State:
        return self._current_state


def main():
    if len(sys.argv) == 4:
        osi_extractor = SynchronOSI3Ectractor(rec_ip_addr=sys.argv[1],
                                              rec_port=int(sys.argv[2]),
                                              ego_id=int(sys.argv[3]))
    elif len(sys.argv) == 6:
        osi_extractor = SynchronOSI3Ectractor(rec_ip_addr=sys.argv[1],
                                              rec_port=int(sys.argv[2]),
                                              ego_id=int(sys.argv[3]),
                                              esmini_ip_addr=sys.argv[4],
                                              esmini_port=int(sys.argv[5]))
    else:
        print(f"Usage:\n{sys.argv[0]} <listen ip> <port> <ego vehicle id> [<esmini ip> <esmini driver port>]")
        sys.exit(1)
    
    with osi_extractor:
        for _ in range(40):
            time.sleep(0.3)
           # osi_extractor.send_empty_update(0)
            osi_extractor.send_driver_update(DriverUpdate(0,0.4, 0.0, 0.3))
            print(osi_extractor.get_next_state())


if __name__ == "__main__":
    main()
