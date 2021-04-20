import importlib.util
import os
from threading import Thread

import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter

from camera.FrameHandler import FrameHandler
from camera.HeadController import HeadController

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
    __head: HeadController

    def __init__(self):
        super().__init__()

        self.__alive = True

        self.__pkg = importlib.util.find_spec("tflite_runtime")

        with open(PATH_TO_LABELS, "r") as f:
            self.__labels = [line.strip() for line in f.readlines()]

        if self.__labels[0] == '???':
            del (self.__labels[0])

        # Then load tensorflow lite model
        self.__interpreter = Interpreter(model_path=PATH_TO_CKPT)
        self.__interpreter.allocate_tensors()

        self.__head = HeadController()

    def run(self) -> None:
        super().run()

        input_details = self.__interpreter.get_input_details()
        output_details = self.__interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        floating_model = (input_details[0]['dtype'] == np.float32)

        input_mean = 127.5
        input_std = 127.5

        frame_rate_calc = 1
        freq = cv2.getTickFrequency()

        # TODO: Paste people detection code
        while (self.__alive):
            t1 = cv2.getTickCount()
            ret, image = self.get_next_frame()
            if ret and image is not None:
                # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                imH, imW, _ = image.shape
                image_resized = cv2.resize(image, (width, height))
                input_data = np.expand_dims(image_resized, axis=0)

                if floating_model:
                    input_data = (np.float32(input_data) - input_mean) / input_std

                self.__interpreter.set_tensor(input_details[0]['index'], input_data)
                self.__interpreter.invoke()

                boxes = self.__interpreter.get_tensor(output_details[0]['index'])[0]
                classes = self.__interpreter.get_tensor(output_details[1]['index'])[0]
                scores = self.__interpreter.get_tensor(output_details[2]['index'])[0]

                frame = image

                for i in range(len(scores)):
                    if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
                        # Get bounding box coordinates and draw box
                        # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                        ymin = int(max(1, (boxes[i][0] * imH)))
                        xmin = int(max(1, (boxes[i][1] * imW)))
                        ymax = int(min(imH, (boxes[i][2] * imH)))
                        xmax = int(min(imW, (boxes[i][3] * imW)))

                        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

                        # Draw label
                        object_name = self.__labels[
                            int(classes[i])]  # Look up object name from "labels" array using class index
                        object_name = self.__labels[
                            int(classes[i])]  # Look up object name from "labels" array using class index
                        xmid = xmin + ((xmax - xmin) / 2)
                        p = 640 - xmid
                        if i == 0:
                            angle = self.__head.find_angle(p * (1 / 64))
                            rot = self.__head.rotate(angle)
                        label = '%s: %d - %d%%' % (object_name, angle, rot)  # Example: 'person: 72%'
                        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
                        label_ymin = max(ymin,
                                         labelSize[1] + 10)  # Make sure not to draw label too close to top of window
                        cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                                      (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),
                                      cv2.FILLED)  # Draw white box to put label text in
                        cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),
                                    2)  # Draw label text

                # Draw framerate in corner of frame
                cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 0), 2, cv2.LINE_AA)

                # All the results have been drawn on the frame, so it's time to display it.
                cv2.imshow('Object detector', frame)

                # Calculate framerate
                t2 = cv2.getTickCount()
                time1 = (t2 - t1) / freq
                frame_rate_calc = 1 / time1

                # Press 'q' to quit
                if cv2.waitKey(1) == ord('q'):
                    break

    def stop(self) -> None:
        self.__alive = False
