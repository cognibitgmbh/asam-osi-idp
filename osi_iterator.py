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

# TODO: Make both files and UDP streams work, even though they don´t work the same way


class BinaryStream(abc.ABC):
    @abc.abstractmethod
    def open(self):
        pass

    @abc.abstractmethod
    def read(self, n: int) -> bytes:
        pass

    @abc.abstractmethod
    def close(self):
        pass


class FileStream(BinaryStream):
    def __init__(self, path):
        self.path = path
        self.file: Optional[TextIO] = None

    def open(self):
        if self.file is None:
            self.file = open(self.path, "rb")

    def read(self, n: int) -> bytes:
        if self.file is None:
            raise RuntimeError("File is not open")
        time.sleep(1)
        return self.file.read(n)

    def close(self):
        if self.file is None:
            raise RuntimeError("File is not open")
        self.file.close()
        self.file = None


class UDPStream(BinaryStream):
    def __init__(self, bind_address: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind_address, port))

    def open(self):
        pass

    def read(self, n: int) -> bytes:
        data, _ = self.socket.recvfrom(n)
        return data

    def close(self):
        self.socket.close()


class OSI3GroundTruthIterator:
    def __init__(self, binary_stream: BinaryStream):
        self.binary_stream = binary_stream

    def __enter__(self):
        self.binary_stream.open()
        return self

    def __exit__(self, exc_type: Optional[type[BaseException]], *args) -> bool:
        self.binary_stream.close()
        return exc_type is None

    def __iter__(self) -> Iterator[GroundTruth]:
        while True:
            message_length_str = self.binary_stream.read(4)
            if len(message_length_str) == 0:
                return
            elif len(message_length_str) != 4:
                raise RuntimeError("Unexpected EOF in OSI3 stream")
            print(message_length_str)
            message_length: int = struct.unpack('<I', message_length_str)[0]
            print(message_length)
            message = GroundTruth()
            message.ParseFromString(self.binary_stream.read(message_length))
            yield message
