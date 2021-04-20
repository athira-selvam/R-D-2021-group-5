from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
import time
from threading import Thread


class HeadController(Thread):
    __angle: float
    __alive: bool

    def __init__(self):
        super().__init__()
        self.__alive = True
        self.__angle = 90

    def run(self) -> None:
        while self.__alive:
            print("Rotating head to %f" % self.__angle)
            kit.servo[5].angle = self.__angle
            time.sleep(0.5)

    def rotate(self, ang: float) -> None:

        self.__angle = ang

    def stop(self):
        self.__alive = False
