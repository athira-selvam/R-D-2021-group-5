import time
from threading import Thread
from typing import Optional

import cv2
import zbar

from camera.FrameHandler import FrameHandler
from camera.QRCodeHandler import QRCodeHandler

TIME_THRESHOLD = 5  # Seconds to wait before a code is detected twice


class CodeReader(Thread, FrameHandler):
    __code_handler: QRCodeHandler
    __scanner: zbar.Scanner

    __alive: bool

    __last_code: Optional[str]
    __last_time: float

    def __init__(self):
        super().__init__()
        self._frames_buffer = []
        self.__alive = True

        self.__last_code = None
        self.__last_time = time.time()

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
                    code: str = result[0].data.decode("utf-8")
                    code = code.lower()
                    # Then check the filtering condition
                    # We check whether the received code is the same as the last one
                    if self.__last_code is not None and self.__last_code == code:
                        # Then check whether the time difference is above the threshold
                        now = time.time()
                        time_diff = now - self.__last_time
                        if time_diff <= TIME_THRESHOLD:
                            # If yes we update the last time and just stop
                            self.__last_time = now
                            time.sleep(0.001)
                            continue
                    # Otherwise let it pass and update filter
                    self.__last_time = time.time()
                    self.__last_code = code
                    print("Code %s passed through" % code)
                    # Eventually notifying the handlers
                    self.__code_handler.handle_code(code)
            time.sleep(0.001)

    def stop(self):
        self.__alive = False

    def set_code_handler(self, code_handler: QRCodeHandler) -> None:
        print("[CODE_READER] Registered new code handler")
        self.__code_handler = code_handler
