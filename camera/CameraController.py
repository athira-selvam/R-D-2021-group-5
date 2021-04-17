import time
from threading import Thread

import cv2

from camera.CodeReader import CodeReader
from camera.PeopleDetector import PeopleDetector


class CameraController(Thread):
    __capture: cv2.VideoCapture
    __code_reader: CodeReader
    __people_detector: PeopleDetector

    __alive: bool

    def __init__(self):
        super().__init__()
        # Initialize the video capture device
        self.__capture = cv2.VideoCapture(0)
        self.__code_reader = CodeReader()
        self.__people_detector = PeopleDetector()
        self.__alive = True

        # Then start the threads
        self.__code_reader.start()

    def run(self) -> None:
        # Here we run the code to get frames
        while self.__alive and self.__capture.isOpened():
            # Get the frame from video capture
            ret, frame = self.__capture.read()

            if not ret:
                continue

            # Transform the image to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # And then pass the frame to the two detectors
            self.__code_reader.handle(gray)
            self.__people_detector.handle(gray)

            time.sleep(0.01)

    def stop(self):
        self.__alive = False
