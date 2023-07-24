from osi_extractor import SynchronOSI3Extractor
from driver import DriverController

RECEIVE_ADDRESS = "0.0.0.0"
RECEIVE_PORT = 48198
EGO_VEHICLE_ID = 0
ESMINI_ADDRESS = "172.29.240.1"
ESMINI_PORT = 49950


def main():
    osi_extractor = SynchronOSI3Extractor(rec_ip_addr=RECEIVE_ADDRESS,
                                          rec_port=RECEIVE_PORT,
                                          ego_id=EGO_VEHICLE_ID,
                                          esmini_ip_addr=ESMINI_ADDRESS,
                                          esmini_port=ESMINI_PORT)
    driver_controller = DriverController(EGO_VEHICLE_ID)
    start = True

    with osi_extractor:
        while True:
            state = osi_extractor.get_next_state()
            driver_update = driver_controller.process_next_state(state)
            osi_extractor.send_driver_update(driver_update)


if __name__ == "__main__":
    main()
