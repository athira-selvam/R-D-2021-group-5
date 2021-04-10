from servo import Servo
import time
import threading

class Stick:
    top_motor = None
    bottom_motor = None
    top_position = [135, 45]
    bottom_position = [35, 90, 145]
    has_stick = None
    tempo = 0  # bpm
    new_tempo = 0
    moving = False
    started_time = 0
    elapsed_time = 0
    duration = 0
    rhythm = "four_four"  # default
	
    def __init__(self, has_stick, initial_tempo, channels):
        self.has_stick = has_stick
        if not has_stick:
        	bottom_position[145, 90, 30]#reflect the movement
        self.tempo = initial_tempo
        self.new_tempo = initial_tempo
        self.top_motor = Servo("top", channels[0], self.top_position[0])
        self.bottom_motor = Servo("bottom", channels[1], self.bottom_position[1])

    def set_rhythm(self, rhythm):
        if rhythm in {"two_two", "four_four", "three_four", "quick_tempo"}:
            self.rhythm = rhythm
        else:
            print("This rhythm is not allowed")

    def two_two(self):
        """Moves the top motor to position[1] => A
           Moves the top motor to position[0] => B
           ALL step has timing 60/tempo"""
           
        self.top_motor.tick_start(self.top_position[1], 60000.0 / float(self.tempo))  # A
        self.top_motor.tick_start(self.top_position[0], 60000.0 / float(self.tempo))  # B
        

    def four_four(self):
        """Moves the top motor to position[1] => A
           Moves the bottom motor to position[2] => B
           Moves the bottom motor to position[0] => C
           Moves both the bottom and the top motor to position[1][0] respectively => D1,D2
           ALL step has timing 60/tempo"""
           
        self.top_motor.tick_start(self.top_position[1], 60000.0 / float(self.tempo))  # A
        self.bottom_motor.tick_start(self.bottom_position[2], 60000.0 / float(self.tempo))  # B
        self.bottom_motor.tick_start(self.bottom_position[0], 60000.0 / float(self.tempo))  # C
        d1 = threading.Thread(target=self.bottom_motor.tick_start,
                              args=(self.bottom_position[1], 60000.0 / float(self.tempo),))  # D1
        d2 = threading.Thread(target=self.top_motor.tick_start,
                              args=(self.top_position[0], 60000.0 / float(self.tempo),))  # D2
        d1.start()
        d2.start()
        d1.join()
        d2.join()

    def three_four(self):
        """Moves the top motor to position[1] => A
           Moves the bottom motor to position[0] => B
           Moves both the bottom and the top motor to position[1][0] respectively => C1,C2
           ALL step has timing 60/tempo"""
        
        self.top_motor.tick_start(self.top_position[1], 60000.0 / float(self.tempo))  # A
        self.bottom_motor.tick_start(self.bottom_position[0], 60000.0 / float(self.tempo))  # B
        c1 = threading.Thread(target=self.bottom_motor.tick_start,
                              args=(self.bottom_position[1], 60000.0 / float(self.tempo),))  # C1
        c2 = threading.Thread(target=self.top_motor.tick_start,
                              args=(self.top_position[0], 60000.0 / float(self.tempo),))  # C2
        c1.start()
        c2.start()
        c1.join()
        c2.join()

    def quick_tempo(self):
        # not implemented yet
        pass

    def animate(self, rythm, duration, initial_tempo):
        self.moving = True
        self.started_time = time.time() * 1000
        self.duration = duration  # in millisecond
        self.tempo = initial_tempo
        self.new_tempo = initial_tempo
        self.elapsed_time = 0
        while self.elapsed_time <= duration:
            if rythm == "two_two":
                self.two_two()
            elif rythm == "four_four":
                self.four_four()
            elif rythm == "three_four":
                self.three_four()
            elif rythm == "quick_tempo":
                self.quick_tempo()  # not implemented yet
            else:
                print("This rythm is not allowed")
            # update the elapsed time
            self.elapsed_time = time.time() * 1000 - self.started_time
            # synchronize the tempo update and make it effective at the next loop
            if self.new_tempo != self.tempo:
                self.tempo = self.new_tempo
	def new_tempo(self, new_tempo):
		self.new_tempo = new_tempo
