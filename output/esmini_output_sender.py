

import socket
import struct

from output.driver_update import DriverUpdate
from output.output_sender import OutputSender
from output.raw_update import RawUpdate


input_modes = {
    'driverInput': 1,
    'stateXYZHPR': 2,
    'stateXYH': 3,
}


class EsminiOutputSender(OutputSender):
    def __init__(self, address: str, baseport: int):
        self.address = address
        self.baseport = baseport

    def open(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_nrs: map[int, int] = {} # TODO This seems usless

    def close(self):
        if self.socket is not None:
            self.socket.close()
    
    def send_empty_update(self, object_id: int):
        # This function can be used, to trigger the next timestep in esmini, without sending payload
        message = struct.pack(
            'ii',
            1,    # version
            0,    # message type = 'NO_INPUT'
        )
        self.socket.sendto(
            message, (self.address, self.baseport + object_id))

    def send_driver_update(self, driver_update: DriverUpdate):
        frame_nr = self.frame_nrs.get(driver_update.id, 0)
        message = struct.pack(
            'iiiiddd',
            1,    # version
            1,    # message type = 'driverInput'
            driver_update.id,    # object ID
            frame_nr,
            driver_update.throttle,
            driver_update.brake,
            driver_update.steer
        )
        self.socket.sendto(
            message, (self.address, self.baseport + driver_update.id))
        self.frame_nrs[driver_update.id] = frame_nr + 1

    def send_raw_update(self, raw_update:  RawUpdate):
        raise NotImplementedError(
            "Only DriverUpdate implemented for Esmini output")
