#!/usr/bin/env python
import time
from easing_functions import *
#from adafruit_servokit import ServoKit


# kit = ServoKit(channels=16)

class Servo:
    angle = 90
    name = "Servo"
    max_angle = 180
    min_angle = 0
    transition_style = "ease_in_out_circular"
    duration = 0  # this is in seconds
    channel = 0
    target_angle = 0
    current_time = 0
    start_value = 0
    change_in_value = 0
    tick_started = False
    tick_start_time = 0
    elapsed_time = 0
    transition = 0

    def tick_start(self, target, duration):
        if not self.tick_started:
            self.tick_started = True
            self.target_angle = target
            self.duration = duration  # in millisecond
            self.elapsed_time = 0
            if self.transition_style == "ease_in_out_circular":
                self.transition = CircularEaseInOut(start=self.angle, end=target, duration=duration)
            # add more if needed

            self.tick_start_time = time.time() * 1000
            while not (self.tick()):
                self.elapsed_time = time.time() * 1000 - self.tick_start_time
            self.tick_started = False
            
        else:
            print("The motor is still moving!")

    def tick(self):
        if self.elapsed_time >= self.duration:
            return True
        cur_angle = self.transition.ease(self.elapsed_time)
        self.angle = cur_angle
        #print(self.name + "\t" + str(cur_angle) + "\t" + str(time.time() * 1000))
        time.sleep(0.01)
        return False

    def __init__(self, name, channel, default_angle):
        self.name = name
        self.channel = channel
        self.angle = default_angle;

    def angle(self, value):
        if 180 >= value >= 0:
            self.angle = value
            # kit.servo[self.__channel].angle=180
        else:
            print("The angle was too small or too large: ", value)

    def plot(self):
        for i in self.data_x:
            print(i)
        print("/n/n")
        for i in self.data_y:
            print(i)
