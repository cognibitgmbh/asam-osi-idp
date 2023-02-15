# This is deprecated

import struct
from typing import Optional, TextIO

from osi3.osi_sensorview_pb2 import SensorView
from osi3.osi_trafficupdate_pb2 import TrafficUpdate
from osi3.osi_hostvehicledata_pb2 import HostVehicleData
from osi3.osi_common_pb2 import Vector3d, Orientation3d
from output.driver_update import DriverUpdate
from output.raw_update import RawUpdate

from output.output_sender import OutputSender


class OsiTraceOutputSender(OutputSender):
    def __init__(self, filename: str):
        self.filename = filename
        self.file: Optional[TextIO] = None

    def open(self):
        if self.file is None:
            self.file = open(self.filename, 'ab')

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None

    def send_driver_input_update(self, driver_update: DriverUpdate):
        traffic_update = TrafficUpdate()

        isu = traffic_update.internal_state.add()
        isu.host_vehicle_id.value = driver_update.id
        isu.vehicle_steering.vehicle_steering_wheel.angle = driver_update.steer
        isu.vehicle_brake_system.pedal_position_brake = driver_update.brake
        isu.vehicle_powertrain.pedal_position_acceleration = driver_update.throttle

        mou = traffic_update.update.add()
        mou.id.value = driver_update.id
        ls = mou.vehicle_classification.light_state
        if driver_update.indicator_state is not None:
            ls.indicator_state = driver_update.indicator_state
        if driver_update.brake_light_state is not None:
            ls.brake_light_state = driver_update.brake_light_state
        if driver_update.front_fog_light is not None:
            ls.front_fog_light = driver_update.front_fog_light
        if driver_update.rear_fog_light is not None:
            ls.rear_fog_light = driver_update.rear_fog_light
        if driver_update.head_light is not None:
            ls.head_light = driver_update.head_light
        if driver_update.high_beam is not None:
            ls.high_beam = driver_update.high_beam
        if driver_update.reversing_light is not None:
            ls.reversing_light = driver_update.reversing_light
        if driver_update.license_plate_illumination_rear is not None:
            ls. license_plate_illumination_rear = driver_update.license_plate_illumination_rear
        if driver_update.emergency_vehicle_illumination is not None:
            ls.emergency_vehicle_illumination = driver_update.emergency_vehicle_illumination
        if driver_update.service_vehicle_illumination is not None:
            ls.service_vehicle_illumination = driver_update.service_vehicle_illumination

        print(traffic_update)
        bytes_buffer = traffic_update.SerializeToString()
        self.file.write(struct.pack("<L", len(bytes_buffer)))
        self.file.write(bytes_buffer)

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
        bytes_buffer = traffic_update.SerializeToString()
        self.file.write(struct.pack("<L", len(bytes_buffer)))
        self.file.write(bytes_buffer)


#with OsiTraceOutputSender("lol.osi") as otos:
#    print(otos)
#    otos.send_driver_update(DriverUpdate(3, 0.1, 0.2, 0.3, indicator_state=1, brake_light_state=2, front_fog_light=3, rear_fog_light=4,
#                                     head_light=5, high_beam=6, reversing_light=1, license_plate_illumination_rear=2, emergency_vehicle_illumination=3, service_vehicle_illumination=4))


#pos = Vector3d(x=0.0,y=1.0,z=2.0)
#orient = Orientation3d(yaw=1, pitch=0.5, roll=-1)
# otos.send_raw_update(RawUpdate(12, pos,orient,indicator_state=1, brake_light_state=2, front_fog_light=3, rear_fog_light=4,
# head_light=5, high_beam=6, reversing_light=1, license_plate_illumination_rear=2, emergency_vehicle_illumination=3, service_vehicle_illumination=4))

