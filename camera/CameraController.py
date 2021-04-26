import time
from threading import Thread

import cv2

from utils.Singleton import Singleton
from camera.CodeReader import CodeReader
from camera.PeopleDetectionHandler import PeopleDetectionHandler
from camera.PeopleDetector import PeopleDetector
from camera.QRCodeHandler import QRCodeHandler


class CameraController(Singleton, Thread):
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
        self.__people_detector.start()

    def run(self) -> None:
        # Here we run the code to get frames
        while self.__alive and self.__capture.isOpened():
            # Get the frame from video capture
            ret, frame = self.__capture.read()
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            if not ret:
                continue

            # And then pass the frame to the two detectors
            self.__code_reader.handle(frame)
            self.__people_detector.handle(frame)

            time.sleep(0.01)

    def stop(self):
        self.__alive = False

    def subscribe_to_qrcode(self, code_handler: QRCodeHandler) -> None:
        self.__code_reader.set_code_handler(code_handler)

    def subscribe_to_people_detection(self, detection_handler: PeopleDetectionHandler) -> None:
        self.__people_detector.set_detection_handler(detection_handler)
