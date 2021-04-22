import enum


class LedAnimation(enum.Enum):
    ANIM_1 = 1,
    ANIM_2 = 2


class LedController:

    def play_animation(self, animation: LedAnimation) -> None:
        """
        Plays the provided LED animation
        :param animation: the identifier of the desired animation
        """
        # TODO: To be implemented

# TODO: To implement


# TODO: Define all the animations
