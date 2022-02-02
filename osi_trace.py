import math
import struct
import sys
from typing import Iterator, Optional, TextIO

from osi3.osi_groundtruth_pb2 import GroundTruth
import sys
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0, "/home/raphael/idp/Clothoids/lib/lib/")

import G2lib



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

    def __exit__(self, exc_type, *args) -> bool:
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
        count = 0
        for message in trace:
            count += 1
            #for m_o in message.moving_object:
            #    print(m_o.assigned_lane_id)
            for l in message.lane:
                cl = l.classification.centerline
                cl_x = [cur.x for cur in cl]
                cl_y = [cur.y for cur in cl]
                splines = G2lib.buildP4(cl_x,cl_y)


                plt.scatter(cl_x, cl_y)
#                plt.plot(cl_y, cl_y)
                plt.plot(splines.X(np.linspace(0,1500,1000)), splines.Y(np.linspace(0,1500,1000)))
            plt.show()
            #print(cl)
            #print(cl_x)
#            print(message.lane_boundary[7])
         #   print(type(message.moving_object))
         #   print(message.lane[0].classification.centerline[0].x)
         #   print(message.lane[0].classification.centerline[0].y)
         #   print(message.lane[0].classification.centerline[1].x)
         #   print(message.lane[0].classification.centerline[1].y)
         #   print(message.lane[0].classification.centerline[2].x)
         #   print(message.lane[0].classification.centerline[2].y)

         #   x1 = message.lane[0].classification.centerline[0].x
         #   y1 = message.lane[0].classification.centerline[0].y
         #   x2 = message.lane[0].classification.centerline[1].x
         #   y2 = message.lane[0].classification.centerline[1].y
         #   x3 = message.lane[0].classification.centerline[2].x
         #   y3 = message.lane[0].classification.centerline[2].y -1000



         #   vec1_x = x2 - x1
         #   vec1_y = y2 - y1
         #   vec2_x = x3 - x2
         #   vec2_y = y3 - y2
         #   angle = math.acos((vec1_x * vec2_x + vec1_y * vec2_y) / (math.sqrt(vec1_x**2 + vec1_y**2) * math.sqrt(vec2_x**2 + vec2_y**2)))
         #   print(angle*360/2/math.pi)
#            print(message)






if __name__ == '__main__':
    main()
