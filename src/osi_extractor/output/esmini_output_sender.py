

import socket
import struct

from .esmini_update import DriverInputUpdate, XYHSpeedSteeringUpdate
from .output_sender import OutputSender


input_modes = {
    'driverInput': 1,
    'stateXYZHPR': 2,
    'stateXYH': 3,
}


class EsminiOutputSender(OutputSender):
    def __init__(self, address: str, baseport: int):
        self.frame_nrs = None
        self.address = address
        self.baseport = baseport

    def open(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_nrs = {}

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

    def send_driver_input_update(self, driver_input_update: DriverInputUpdate):
        frame_nr = self.frame_nrs.get(driver_input_update.id, 0)
        message = struct.pack(
            'iiiiddd',
            1,    # version
            1,    # message type = 'driverInput'
            driver_input_update.id,    # object ID
            frame_nr,
            driver_input_update.throttle,
            driver_input_update.brake,
            driver_input_update.steer
        )
        self.socket.sendto(
            message, (self.address, self.baseport + driver_input_update.id))
        self.frame_nrs[driver_input_update.id] = frame_nr + 1

    def send_xyh_speed_steering_update(self, xyh_speed_steering_update: XYHSpeedSteeringUpdate):
        frame_nr = self.frame_nrs.get(xyh_speed_steering_update.id, 0)
        message = struct.pack(
            'iiiidddddB',
            1,    # version
            input_modes["stateXYH"],
            xyh_speed_steering_update.id,    # object ID
            frame_nr,
            xyh_speed_steering_update.x,
            xyh_speed_steering_update.y,
            xyh_speed_steering_update.h,
            xyh_speed_steering_update.speed,
            xyh_speed_steering_update.steering_wheel_angle,
            1 if xyh_speed_steering_update.dead_reckon else 0
        )
        self.socket.sendto(
            message, (self.address, self.baseport + xyh_speed_steering_update.id))
        self.frame_nrs[xyh_speed_steering_update.id] = frame_nr + 1

