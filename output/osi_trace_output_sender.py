import struct
from typing import Optional, TextIO

from osi3.osi_sensorview_pb2 import SensorView
from osi3.osi_trafficupdate_pb2 import TrafficUpdate
from osi3.osi_common_pb2 import Vector3d, Orientation3d
from driver_update import DriverUpdate
from raw_update import RawUpdate

from output_sender import OutputSender


class OsiTraceOutputSender(OutputSender):
    def __init__(self, filename: str):
        self.filename = filename
        self.file: Optional[TextIO] = None

    def open(self):
        if self.file is None:
            self.file = open(self.path, 'ab')

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None

    def send_driver_update(self, DriverUpdate):
        pass
        sensorview = SensorView()
        sv_ground_truth = sensorview.global_ground_truth
        sv_ground_truth.version.version_major = 3
        sv_ground_truth.version.version_minor = 4
        sv_ground_truth.version.version_patch = 0
        sv_ground_truth.timestamp.seconds = 0
        sv_ground_truth.timestamp.nanos = 0
        moving_object = sv_ground_truth.moving_object.add()
        moving_object.id.value = 42
        # Generate 1000 OSI messages for a duration of 10 seconds    
        for i in range(1000):
            # Increment the time
            if sv_ground_truth.timestamp.nanos > 1000000000:
                sv_ground_truth.timestamp.seconds += 1
                sv_ground_truth.timestamp.nanos = 0
            sv_ground_truth.timestamp.nanos += NANO_INCREMENT
            moving_object.vehicle_classification.type = 2
            moving_object.base.dimension.length = MOVING_OBJECT_LENGTH
            moving_object.base.dimension.width = MOVING_OBJECT_WIDTH
            moving_object.base.dimension.height = MOVING_OBJECT_HEIGHT
            moving_object.base.position.x += 0.5
            moving_object.base.position.y = 0.0
            moving_object.base.position.z = 0.0
            moving_object.base.orientation.roll = 0.0
            moving_object.base.orientation.pitch = 0.0
            moving_object.base.orientation.yaw = 0.0
            """Serialize"""
            bytes_buffer = sensorview.SerializeToString()
            f.write(struct.pack("<L", len(bytes_buffer)))
            f.write(bytes_buffer)
        f.close()


    def send_raw_update(self, raw_update:  RawUpdate):
        traffic_update = TrafficUpdate()
        print(traffic_update.update)
        mou = traffic_update.update.add()

        mou.id.value = raw_update.id
        mou.base.position.x = raw_update.position.x
        mou.base.position.y = raw_update.position.y
        mou.base.position.z = raw_update.position.z

        mou.base.orientation.yaw = raw_update.orientation.yaw
        mou.base.orientation.pitch = raw_update.orientation.pitch
        mou.base.orientation.roll = raw_update.orientation.roll

        ls = mou.vehicle_classification.light_state
        if raw_update.indicator_state is not None:
            ls.indicator_state = raw_update.indicator_state
        if raw_update.brake_light_state is not None:
            ls.brake_light_state = raw_update.brake_light_state
        if raw_update.front_fog_light is not None:
            ls.front_fog_light = raw_update.front_fog_light
        if raw_update.rear_fog_light is not None:
            ls.rear_fog_light = raw_update.rear_fog_light
        if raw_update.head_light is not None:
            ls.head_light = raw_update.head_light
        if raw_update.high_beam is not None:
            ls.high_beam = raw_update.high_beam
        if raw_update.reversing_light is not None:
            ls.reversing_light = raw_update.reversing_light
        if raw_update.license_plate_illumination_rear is not None:
            ls. license_plate_illumination_rear = raw_update.license_plate_illumination_rear
        if raw_update.emergency_vehicle_illumination is not None:
            ls.emergency_vehicle_illumination = raw_update.emergency_vehicle_illumination
        if raw_update.service_vehicle_illumination is not None:
            ls.service_vehicle_illumination = raw_update.service_vehicle_illumination

        print(traffic_update)

otos = OsiTraceOutputSender("lol.osi")
pos = Vector3d(x=0.0,y=1.0,z=2.0)
orient = Orientation3d(yaw=1, pitch=0.5, roll=-1)
otos.send_raw_update(RawUpdate(12, pos,orient,indicator_state=1, brake_light_state=2, front_fog_light=3, rear_fog_light=4, 
head_light=5, high_beam=6, reversing_light=1, license_plate_illumination_rear=2, emergency_vehicle_illumination=3, service_vehicle_illumination=4))

