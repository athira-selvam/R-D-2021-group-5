from stick import Stick
import time
from math import floor, ceil

class StickManager:
    left_stick = None
    right_stick = None

    def __init__(self):
        self.left_stick = Stick(True, 80, [0, 1])
        self.right_stick = Stick(False, 80, [2, 3])
    
    def start_animation(self, rhythm, initial_tempo, duration, synch_interval):
        self.synchronize(synch_interval) #synchronization
        self.left_stick.start_animate(rhythm, duration, initial_tempo)
        self.right_stick.start_animate(rhythm, duration, initial_tempo)
    
    def stop_animation(self):
        self.left_stick.stop_animate()
        self.right_stick.stop_animate()
    
    def synchronize(self, synch_interval):
        if synch_interval == 0:
            return
        next_second = ceil(time.time())*1000
        next_tick = next_second + (next_second%synch_interval)
        print("synch at: " + next_tick)
        wait_for = (next_tick - time.time()*1000)/1000
        if wait_for > 0:
            time.sleep(wait_for)
    
    def change_tempo(self, new_tempo):
        self.left_stick.set_new_tempo(new_tempo)
        self.right_stick.set_new_tempo(new_tempo)
    
    def change_style(self, new_style):
        self.left_stick.set_rhythm(new_style)
        self.right_stick.set_rhythm(new_style)
    
