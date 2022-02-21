import abc
import socket
import struct
import time
from typing import Iterator, Optional, TextIO

from osi3.osi_groundtruth_pb2 import GroundTruth


# UDP does not work like files:
# - a single message can arrive as several datagrams
# - all but the last datagram start with a 4 byte sequence number, starting at 1
#   - the last datagram has a negative sequence number (e.g. -9 if 8 came before)
# - sequence number is always followed by a 4 byte length
# - this describes the length of the actual payload
#   - seemingly, this is 8192 for everything but the last datagram

class OSI3GroundTruthIterator(abc.ABC):
    
    @abc.abstractmethod
    def __enter__(self):
        pass

    @abc.abstractmethod
    def __exit__(self, exc_type, *args) -> bool:
        pass

    @abc.abstractmethod
    def __iter__(self) -> Iterator[GroundTruth]:
        pass

class UDPGroundTruthIterator(OSI3GroundTruthIterator):
    
    def __init__(self, bind_address: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind_address, port))

    def read(self, n: int = 2**14) -> bytes:
        data = self.socket.recv(n)
        return data

    def parse_int(self, data: bytes) -> int:
        return int.from_bytes(data, byteorder='little', signed=True)

    def __enter__(self):
        pass

    def __exit__(self, exc_type, *args) -> bool:
        self.socket.close()
        return exc_type is None

    def __iter__(self) -> Iterator[GroundTruth]:
        while True:
            expected_slice_id : int = 1
            message_bytes = b""
            while True:
                udp_package = self.read()
                slice_id: int = self.parse_int(udp_package[0:4]) 
                last_slice = (slice_id == -expected_slice_id)
                slice_id = abs(slice_id)
                if slice_id == expected_slice_id:
                    slice_length: int = self.parse_int(udp_package[4:8])
                    if slice_length + 8 != len(udp_package):
                        print("throwing away current groundtruth bytes")
                        print("length field does not conform with udp length")
                        break
                    message_bytes += udp_package[8:] 
                else:
                    print("throwing away current groundtruth bytes")
                    print("received slice id:" + str(slice_id) + "  expected:" + str(expected_slice_id))
                    break
                if last_slice:
                    message = GroundTruth()
                    message.ParseFromString(message_bytes)
                    yield message
                    break
                expected_slice_id += 1


class FileGroundTruthIterator(OSI3GroundTruthIterator):
    def __enter__(self):
        if self.file is None:
            self.file = open(self.path, "rb")

    def __exit__(self, exc_type, *args) -> bool:
        if self.file is None:
            raise RuntimeError("File is not open")
        self.file.close()
        self.file = None
        return exc_type is None

    def __iter__(self) -> Iterator[GroundTruth]:
        while True:
            message_length_str = self.read(4)
            if len(message_length_str) == 0:
                return
            elif len(message_length_str) != 4:
                raise RuntimeError("Unexpected EOF in OSI3 stream")
            print(message_length_str)
            message_length: int = struct.unpack('<I', message_length_str)[0]
            print(message_length)
            message = GroundTruth()
            message.ParseFromString(self.read(message_length))
            yield message

    def __init__(self, path):
        self.path = path
        self.file: Optional[TextIO] = None


    def read(self, n: int) -> bytes:
        if self.file is None:
            raise RuntimeError("File is not open")
        time.sleep(1)
        return self.file.read(n)

