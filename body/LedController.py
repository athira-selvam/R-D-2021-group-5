import enum
import random
from threading import Thread

import board
import neopixel
import time

from utils.Singleton import Singleton

LED_COUNT = 19

EYE_INITIAL_LED = 4


class LedAnimation(enum.Enum):
    ANIM_OFF = 0,
    ANIM_IDLE = 1,
    ANIM_SUCCESS = 2,
    ANIM_SUCCESS_FILL = 3
    ANIM_ERROR = 4,
    ANIM_MUSIC = 5,
    ANIM_INSTRUMENT = 6,
    ANIM_EYE_0 = 7,
    ANIM_EYE_1 = 8,
    ANIM_EYE_2 = 9,


class LedController(Singleton, Thread):
    __pixels: None
    __animation: LedAnimation
    __alive: bool

    def __init__(self):
        super().__init__()
        self.__pixels = neopixel.NeoPixel(board.D18, 40, brightness=1.0, auto_write=False)
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
            elif self.__animation == LedAnimation.ANIM_SUCCESS_FILL:
                self.success_fill()
            elif self.__animation == LedAnimation.ANIM_ERROR:
                self.error_animation()
            elif self.__animation == LedAnimation.ANIM_INSTRUMENT:
                self.instrument_animation()
            elif self.__animation == LedAnimation.ANIM_EYE_0:
                self.eye_animation(0)
            elif self.__animation == LedAnimation.ANIM_EYE_1:
                self.eye_animation(1)
            elif self.__animation == LedAnimation.ANIM_EYE_2:
                self.eye_animation(2)
            else:
                print("Unsupported animation")
            time.sleep(0.0001)

    def stop(self):
        self.__alive = False

    def eye_animation(self, eye_index):
        self.__pixels.fill((0, 0, 0))
        self.__pixels[EYE_INITIAL_LED + eye_index] = (250, 166, 51)
        self.__pixels[EYE_INITIAL_LED + eye_index + 5] = (250, 166, 51)
        self.__pixels.write()
        time.sleep(0.1)

    def off_animation(self):
        self.__pixels.fill((0, 0, 0))
        self.__pixels.show()

    def idle_animation(self):
        self.__pixels.fill((255, 255, 255))
        self.__pixels.show()
        time.sleep(random.randint(1, 3))
        for i in range(255, 0, -2):
            self.__pixels.fill((i, i, i))
            self.__pixels.show()
            time.sleep(0.00001)
        for i in range(0, 255, 2):
            self.__pixels.fill((i, i, i))
            self.__pixels.show()
            time.sleep(0.00001)

    def success_animation(self):
        for i in range(20):
            self.__pixels[19 - i] = (0, 255, 0)
            self.__pixels[20 + i] = (0, 255, 0)
            self.__pixels.show()
            time.sleep(0.05)
        self.__animation = LedAnimation.ANIM_SUCCESS_FILL
        time.sleep(0.01)

    def success_fill(self):
        self.__pixels.fill((0, 255, 0))
        self.__pixels.show()
        time.sleep(random.random() * 0.75 + 0.33)
        for i in range(255, 55, -2):
            self.__pixels.fill((0, i, 0))
            self.__pixels.show()
            time.sleep(0.00001)
        for i in range(55, 255, 2):
            self.__pixels.fill((0, i, 0))
            self.__pixels.show()
            time.sleep(0.00001)

    def error_animation(self):
        for i in range(255):
            self.__pixels.fill((i, 0, 0))
            self.__pixels.show()
            time.sleep(0.00001)
        for i in range(255, 0, -1):
            self.__pixels.fill((i, 0, 0))
            self.__pixels.show()
            time.sleep(0.00001)

    def instrument_animation(self):
        self.center_to_outside(255, 0, 0, 1, 60 / 1000, 460 / 1000)
        self.center_to_outside(0, 255, 0, 1, 60 / 1000, 460 / 1000)
        self.center_to_outside(0, 0, 255, 1, 60 / 1000, 460 / 1000)

    def center_to_outside(self, red: int, green: int, blue: int, size: int, speed_delay: float,
                          return_delay: float) -> None:
        fade = 255 / ((LED_COUNT - size) / 2)
        for i in range(int((LED_COUNT - size) / 2), -1, -1):
            self.__pixels.fill((0, 0, 0))
            self.__pixels[i] = (red / 10, green / 10, blue / 10)
            for j in range(1, size + 1, 1):
                self.__pixels[i + j] = (red, green, blue)
            self.__pixels[i + size + 1] = (red / 10, green / 10, blue / 10)
            self.__pixels[LED_COUNT - i] = (red / 10, green / 10, blue / 10)
            for j in range(1, size + 1, 1):
                self.__pixels[LED_COUNT - i - j] = (red, green, blue)
            self.__pixels[LED_COUNT - i - size - 1] = (red / 10, green / 10, blue / 10)
            self.__pixels.show()
            if red >= fade:
                red = red - fade
            if green >= fade:
                green = green - fade
            if blue >= fade:
                blue = blue - fade
            time.sleep(speed_delay)
        time.sleep(return_delay)
