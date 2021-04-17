import importlib.util
import os
import time
from threading import Thread

import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter

from camera.FrameHandler import FrameHandler

GRAPH_NAME = "detect.tflite"
LABELMAP_NAME = "labelmap.txt"
min_conf_threshold = float(0.5)

CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH, GRAPH_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH, LABELMAP_NAME)


class PeopleDetector(Thread, FrameHandler):
    """
    An interface describing an object that can handle the result of the people detection process
    """

    __pkg: object
    __labels: list
    __interpreter: Interpreter

    __height: float
    __width: float

    __alive: bool

    def __init__(self):
        super().__init__()
        self._frames_buffer = []

        self.__alive = True

        self.__pkg = importlib.util.find_spec("tflite_runtime")

        with open(PATH_TO_LABELS, "r") as f:
            self.__labels = [line.strip() for line in f.readlines()]

        if self.__labels[0] == '???':
            del (self.__labels[0])

        # Then load tensorflow lite model
        self.__interpreter = Interpreter(model_path=PATH_TO_CKPT)
        self.__interpreter.allocate_tensors()

    def run(self) -> None:
        super().run()

        input_details = self.__interpreter.get_input_details()
        output_details = self.__interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        floating_model = (input_details[0]['dtype'] == np.float32)

        input_mean = 127.5
        input_std = 127.5

        # TODO: Paste people detection code
        while (self.__alive):
            ret, image = self.get_next_frame()
            if ret and image is not None:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                imH, imW, _ = image.shape
                image_resized = cv2.resize(image_rgb, (width, height))
                input_data = np.expand_dims(image_resized, axis=0)

                if floating_model:
                    input_data = (np.float32(input_data) - input_mean) / input_std

                self.__interpreter.set_tensor(input_details[0]['index'], input_data)
                self.__interpreter.invoke()

                classes = self.__interpreter.get_tensor(output_details[1]['index'])[0]
                scores = self.__interpreter.get_tensor(output_details[2]['index'])[0]

                for i in range(len(scores)):
                    if (scores[i] > min_conf_threshold) and (scores[i] <= 1.0):
                        object_name = self.__labels[int(classes[i])]
                        if object_name == "person":
                            print("Person detected!, frames still to consider: %d" % len(self._frames_buffer))
                            # TODO: Notify the handler about succesful detection

    def stop(self) -> None:
        self.__alive = False
