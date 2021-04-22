import enum
import board
import neopixel

class LedController:
    pixels = neopixel.NeoPixel(board.D18, 40, brightness=1.0, auto_write=True)
    
    def play_animation(self, animation: enum.Enum) -> None:
        """
        Plays the provided LED animation
        :param animation: the identifier of the desired animation
        """
        # TODO: To be implemented
    
    def idle_animation(self):
        for i in range(255):
            pixels.fill((i, i, i))
            time.sleep(10)
        for i in range(255,0,-1):
            pixels.fill((i, i, i))
            time.sleep(10)
    
    def success_animation(self):
        for i in range(255):
            pixels.fill((0, i, 0))
            time.sleep(10)
        for i in range(255,0,-1):
            pixels.fill((0, i, 0))
            time.sleep(10)
    
    def error_animation(self):
        for i in range(255):
            pixels.fill((i, 0, 0))
            time.sleep(10)
        for i in range(255,0,-1):
            pixels.fill((i, 0, 0))
            time.sleep(10)

class LedAnimation(enum.Enum):
	ANIM_IDLE = 0,
    ANIM_SUCCESS = 1,
    ANIM_ERROR = 2,
    ANIM_MUSIC = 3,
    ANIM_INSTRUMENT = 4
    

    # TODO: Define all the animations
