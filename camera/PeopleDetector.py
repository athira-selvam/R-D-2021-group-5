from threading import Thread

from camera.FrameHandler import FrameHandler


class PeopleDetector(Thread, FrameHandler):
    """
    An interface describing an object that can handle the result of the people detection process
    """

    def __init__(self):
        super().__init__()
        self._frames_buffer = []

    def run(self) -> None:
        super().run()
        # TODO: Paste people detection code


