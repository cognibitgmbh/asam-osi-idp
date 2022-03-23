from abc import ABC, abstractclassmethod


class OutputSender(ABC):
    @abstractclassmethod
    def send_driver_update(self, DriverUpdate):
        pass

    @abstractclassmethod
    def send_raw_update(self, DriverUpdate):
        pass
