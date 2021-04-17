from abc import ABC


class FrameHandler(ABC):
    """
    A class representing an object that can handle a stream of frames
    """

    _frames_buffer: list

    def handle(self, frame) -> None:
        """
        Handles the provided frame
        :param frame: the object representing the frame
        """
        # Append the frame to the buffer
        self._frames_buffer.append(frame)

    def get_next_frame(self) -> (bool, object):
        """
        Retrieves the next frame in the buffer
        :return: A tuple, where the first element tells whether a frame is available and the second is the actual frame
        """
        if len(self._frames_buffer) == 0:
            return False, None
        else:
            return True, self._frames_buffer.pop(0)
