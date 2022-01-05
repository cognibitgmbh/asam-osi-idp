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
            yield GroundTruth().ParseFromString(self.file.read(message_length))


def main():
    if len(sys.argv) != 2:
        print(f"Usage:\n{sys.argv[0]} <osi_trace_file>")
        sys.exit(1)
    with OSI3GroundTruthTrace(sys.argv[1]) as trace:
        count = 0
        for _ in trace:
            count += 1
        print(count)


if __name__ == '__main__':
    main()
