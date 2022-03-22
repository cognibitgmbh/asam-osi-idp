from udp_osi_common import *

class VehicleUpdateSenderHandler:
    base_port: int
    addr: str
    frame_counter
    def send_vehicle_update(v_u: VehicleUpdate):

