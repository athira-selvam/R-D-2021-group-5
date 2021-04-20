from threading import Thread

import cv2
import zbar

from camera.FrameHandler import FrameHandler
from camera.QRCodeHandler import QRCodeHandler


class CodeReader(Thread, FrameHandler):
    __code_handler: QRCodeHandler
    __scanner: zbar.Scanner

    __alive: bool

    # TODO: Add reference to code handlers

    def __init__(self):
        super().__init__()
        self._frames_buffer = []
        self.__alive = True

        self.__scanner = zbar.Scanner()

    def run(self) -> None:
        super().run()
        while self.__alive:
            # Get the next frame from the buffer
            ret, frame = self.get_next_frame()
            if ret and frame is not None:
                # First transform the frame to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                result = self.__scanner.scan(gray)
                if len(result) != 0:
                    # Retrieve the actual content and decode it into a string
                    code = result[0].data.decode("utf-8")
                    self.__code_handler.handle_code(code)
                # TODO: Emit the code when something is detected

    def stop(self):
        self.__alive = False

    def set_code_handler(self, code_handler: QRCodeHandler) -> None:
        print("[CODE_READER] Registered new code handler")
        self.__code_handler = code_handler
