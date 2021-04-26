from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
import time
from threading import Thread


class HeadController(Thread):
    __angle: float
    __alive: bool
    __enable: bool

    def __init__(self):
        super().__init__()
        self.__alive = True
        self.__enable = True
        self.__angle = 90

    def run(self) -> None:
        while self.__alive:
            if self.__enable:
                print("Rotating head to %f" % self.__angle)
                kit.servo[0].angle = self.__angle
                self.__enable = False
                time.sleep(1)

    def rotate(self, ang: float) -> None:
        if 0 <= ang <= 180:
            self.__angle = ang
            self.__enable = True

    def stop(self):
        self.__alive = False
