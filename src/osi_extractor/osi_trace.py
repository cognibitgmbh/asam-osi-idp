import struct
import sys
from typing import Iterator, Optional, TextIO

from osi3.osi_groundtruth_pb2 import GroundTruth


class OSI3GroundTruthTrace:
    def __init__(self, path: str):
        self.path = path
        self.file: Optional[TextIO] = None

    def open(self):
        if self.file is None:
            self.file = open(self.path, 'rb')

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type: Optional[type[BaseException]], *args) -> bool:
        self.close()
        return exc_type is None

    def __iter__(self) -> Iterator[GroundTruth]:
        self.open()
        while True:
            message_length_str = self.file.read(4)
            if len(message_length_str) == 0:
                return
            elif len(message_length_str) != 4:
                raise RuntimeError("Unexpected EOF in OSI3 trace file")
            message_length: int = struct.unpack('<I', message_length_str)[0]
            message = GroundTruth()
            message.ParseFromString(self.file.read(message_length))
            yield message


def main():
    if len(sys.argv) != 2:
        print(f"Usage:\n{sys.argv[0]} <osi_trace_file>")
        sys.exit(1)
    with OSI3GroundTruthTrace(sys.argv[1]) as trace:
        message = trace.__iter__().__next__()
        for object in message.moving_object:
            print(object)
            #if object.id == message.host_vehicle_id:
            #    print(len(object.assigned_lane_id))
            #    for lane_id in object.assigned_lane_id:
            #        print(lane_id.value)
        #for lane in message.lane:
        #    print(len(lane.classification.centerline))


if __name__ == '__main__':
    main()
