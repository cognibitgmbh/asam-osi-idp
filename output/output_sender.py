from abc import ABC, abstractmethod

from output.esmini_update import DriverInputUpdate, XYHSpeedSteeringUpdate


class OutputSender(ABC):
    @classmethod
    @abstractmethod
    def send_driver_input_update(self, driver_input_update: DriverInputUpdate):
        pass

    @classmethod
    @abstractmethod
    def send_xyh_speed_steering_update(self, xyh_speed_steering_update: XYHSpeedSteeringUpdate):
        pass
