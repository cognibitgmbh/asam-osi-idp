import sys
import threading

from osi3.osi_groundtruth_pb2 import GroundTruth

from  osi_iterator import OSI3GroundTruthIterator, UDPStream


class OSI3Extractor:
    def __init__(self, ip_addr: str, port: int = 48198):
        udp_stream = UDPStream(ip_addr, port)
        self.ground_truth_iterator = OSI3GroundTruthIterator(udp_stream)
        self.thread = threading.Thread(target=self.thread_target)
    
    def start(self) -> threading.Thread:
        self.thread.start()
        return self.thread

    def thread_target(self):
        for ground_truth in self.ground_truth_iterator:
            for object in ground_truth.moving_object:
                if object.id == ground_truth.host_vehicle_id:
                    print(object.base.orientation)
                    break
            print("----------------------------------------------------")


def main():
    if len(sys.argv) != 2:
        print(f"Usage:\n{sys.argv[0]} <port>")
        sys.exit(1)
    osi_extractor = OSI3Extractor("127.0.0.1", int(sys.argv[1]))
    osi_extractor.start()


if __name__ == "__main__":
    main()