import enum


class LedController:

    def play_animation(self, animation: enum.Enum) -> None:
        """
        Plays the provided LED animation
        :param animation: the identifier of the desired animation
        """
        # TODO: To be implemented

# TODO: To implement


class LedAnimation(enum.Enum):
    ANIM_1 = 1,
    ANIM_2 = 2

    # TODO: Define all the animations
