import math
import sys

from .osi_extractor import SynchronOSI3Extractor
from .output.esmini_update import XYHSpeedSteeringUpdate

def main():
    if len(sys.argv) == 4:
        osi_extractor = SynchronOSI3Extractor(rec_ip_addr=sys.argv[1],
                                              rec_port=int(sys.argv[2]),
                                              ego_id=int(sys.argv[3]))
    elif len(sys.argv) == 6:
        osi_extractor = SynchronOSI3Extractor(rec_ip_addr=sys.argv[1],
                                              rec_port=int(sys.argv[2]),
                                              ego_id=int(sys.argv[3]),
                                              esmini_ip_addr=sys.argv[4],
                                              esmini_port=int(sys.argv[5]))
    else:
        print(f"Usage:\n{sys.argv[0]} <listen ip> <port> <ego vehicle id> [<esmini ip> <esmini driver port>]")
        sys.exit(1)

    with osi_extractor:
        x = 0.0
        y = 0.0
        h = 0.0
        speed = 200.0
        steering = 0.5

        for _ in range(400000):
            x += 0.01
            y += 0.001
            h += 0.0001
            h = h % (2 * math.pi)
            # time.sleep(0.3)
            # osi_extractor.send_empty_update(0)
            #      osi_extractor.send_driver_update(DriverInputUpdate(0,0.4, 0.0, 0.3))
            osi_extractor.send_xyh_speed_steering_update(XYHSpeedSteeringUpdate(0, x, y, h, speed, steering, True))
            m = osi_extractor.get_next_state().moving_objects[0]
            print(f"{m.road_id}")


if __name__ == "__main__":
    main()
