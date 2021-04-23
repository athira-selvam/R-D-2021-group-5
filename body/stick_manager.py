from body.Synchronized import Synchronized
from body.stick import Stick


class StickManager(Synchronized):
    left_stick = None
    right_stick = None

    def __init__(self):
        self.left_stick = Stick(True, 80, [0, 1])
        self.right_stick = Stick(False, 80, [2, 3])

    def start_animation(self, rhythm, initial_tempo, duration, synch_interval):
        self.synchronize(synch_interval)  # synchronization
        self.left_stick.start_animate(rhythm, duration, initial_tempo)
        self.right_stick.start_animate(rhythm, duration, initial_tempo)

    def stop_animation(self):
        self.left_stick.stop_animate()
        self.right_stick.stop_animate()

    def change_tempo(self, new_tempo):
        self.left_stick.set_new_tempo(new_tempo)
        self.right_stick.set_new_tempo(new_tempo)

    def change_style(self, new_style):
        self.left_stick.set_rhythm(new_style)
        self.right_stick.set_rhythm(new_style)
