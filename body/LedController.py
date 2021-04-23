import enum
import random
from threading import Thread

import board
import neopixel
import time

from Singleton import Singleton


class LedAnimation(enum.Enum):
    ANIM_OFF = 0,
    ANIM_IDLE = 1,
    ANIM_SUCCESS = 2,
    ANIM_ERROR = 3,
    ANIM_MUSIC = 4,
    ANIM_INSTRUMENT = 5


class LedController(Singleton, Thread):
    __pixels: None
    __animation: LedAnimation
    __alive: bool

    def __init__(self):
        super().__init__()
        self.__pixels = neopixel.NeoPixel(board.D18, 40, brightness=1.0, auto_write=True)
        # And set the blank animation as the default one
        self.__animation = LedAnimation.ANIM_OFF
        self.__alive = True

    def play_animation(self, animation: LedAnimation) -> None:
        """
        Plays the provided LED animation
        :param animation: the identifier of the desired animation
        """
        self.__animation = animation

    def run(self) -> None:
        while self.__alive:
            if self.__animation == LedAnimation.ANIM_OFF:
                self.off_animation()
            elif self.__animation == LedAnimation.ANIM_IDLE:
                self.idle_animation()
            elif self.__animation == LedAnimation.ANIM_SUCCESS:
                self.success_animation()
            elif self.__animation == LedAnimation.ANIM_ERROR:
                self.error_animation()
            else:
                print("Unsupported animation")
            time.sleep(0.0001)

    def stop(self):
        self.__alive = False

    def off_animation(self):
        self.__pixels.fill((0, 0, 0))

    def idle_animation(self):
        self.__pixels.fill((255, 255, 255))
        time.sleep(random.randint(1, 3))
        for i in range(255, 0, -2):
            self.__pixels.fill((i, i, i))
            time.sleep(0.00001)
        for i in range(0, 255, 2):
            self.__pixels.fill((i, i, i))
            time.sleep(0.00001)

    def success_animation(self):
        for i in range(255):
            self.__pixels.fill((0, i, 0))
            time.sleep(0.00001)
        for i in range(255, 0, -1):
            self.__pixels.fill((0, i, 0))
            time.sleep(0.00001)

    def error_animation(self):
        for i in range(255):
            self.__pixels.fill((i, 0, 0))
            time.sleep(0.00001)
        for i in range(255, 0, -1):
            self.__pixels.fill((i, 0, 0))
            time.sleep(0.00001)
