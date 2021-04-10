import time
from servo import Servo

motor = Servo("top_left", 0, 90)
motor.tick_start(180, 1000);
