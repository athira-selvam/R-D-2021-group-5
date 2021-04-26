import importlib.util
import math
import os
from abc import ABC, abstractmethod
from threading import Thread
from typing import Optional
from random import randint

import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter

from body.LedController import LedController, LedAnimation
from camera.FrameHandler import FrameHandler
from camera.HeadController import HeadController
from camera.PeopleDetectionHandler import PeopleDetectionHandler
from camera.centroidtracker import CentroidTracker

GRAPH_NAME = "detect.tflite"
LABELMAP_NAME = "labelmap.txt"
min_conf_threshold = float(0.5)

CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH, GRAPH_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH, LABELMAP_NAME)


class DetectionState(ABC):
    _detection_handler: Optional[PeopleDetectionHandler]
    _emitted: bool

    def __init__(self, detection_handler: PeopleDetectionHandler):
        self._detection_handler = detection_handler
        self._emitted = False

    def set_detection_handler(self, detection_handler: PeopleDetectionHandler):
        self._detection_handler = detection_handler
        self.notify_handler()

    @abstractmethod
    def notify_handler(self):
        pass

    @abstractmethod
    def on_detection_result(self, person_detected: bool):
        """
        Determines the state transition based on the detection result
        :param person_detected: True if a person is present in the frame, False otherwise
        :return: The object representing the new state
        """
        pass


# FSM to handle detection results

class PersonPresentState(DetectionState):

    def __init__(self, detection_handler: PeopleDetectionHandler):
        super().__init__(detection_handler)
        # emit the detection event
        if self._detection_handler is not None:
            self.notify_handler()

    def on_detection_result(self, person_detected: bool):
        if not person_detected:
            return PersonNotPresentState(self._detection_handler)
        else:
            return self

    def notify_handler(self):
        if not self._emitted:
            self._emitted = True
            self._detection_handler.handle_person(True)


class PersonNotPresentState(DetectionState):
    __detected_frames: int  # The number of frames a person has been detected so far

    def __init__(self, detection_handler: PeopleDetectionHandler):
        super().__init__(detection_handler)
        self.__detected_frames = 0
        if self._detection_handler is not None:
            self.notify_handler()

    def on_detection_result(self, person_detected: bool):
        if person_detected:
            return PersonPresentState(self._detection_handler)
        else:
            return self

    def notify_handler(self):
        if not self._emitted:
            self._emitted = True
            self._detection_handler.handle_person(False)


# TODO: Make the state emit the event


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

    __detection_handler: Optional[PeopleDetectionHandler]
    __tracker: CentroidTracker

    __detection_state: DetectionState  # Keep a reference ot the state

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
        self.__head.start()
        self.__tracker = CentroidTracker()

        self.__detection_handler = None

        self.__detection_state = PersonNotPresentState(self.__detection_handler)

    def run(self) -> None:
        super().run()

        input_details = self.__interpreter.get_input_details()
        output_details = self.__interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        floating_model = (input_details[0]['dtype'] == np.float32)

        input_mean = 127.5
        input_std = 127.5

        counter = {}ù
        rot = 0
        

        while self.__alive:
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
                frame_w = frame.shape[1]
                frame_h = frame.shape[0]

                r = []
                track_id = 0

                for i in range(len(scores)):
                    object_name = self.__labels[int(classes[i])]
                    if (object_name == "person" and scores[i] > min_conf_threshold) and (scores[i] <= 1.0):
                        rects = []
                        ymin = int(max(1, (boxes[i][0] * imH)))
                        rects.append(ymin)
                        xmin = int(max(1, (boxes[i][1] * imW)))
                        rects.append(xmin)
                        ymax = int(min(imH, (boxes[i][2] * imH)))
                        rects.append(ymax)
                        xmax = int(min(imW, (boxes[i][3] * imW)))
                        rects.append(xmax)
                        rects = [xmin, ymin, xmax, ymax]

                        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 3)

                        val = np.array(rects)
                        r.append(val.astype("int"))

                # ---------------------------------Head random rotation, rotate to 90 if any people detected---------------------------------#
                left = randint(0, 85)
                right = randint(95,180)
                if len(r) > 0:
                    self.__head.rotate(90)

                else:
                    self.__head.rotate(left if rot == 0 else right)
                    # Here we tell the detection state that no person is present
                    self.__detection_state = self.__detection_state.on_detection_result(False)
                    # And then toggle the rotation
                    rot = 1 if rot == 0 else 1

                # --------------------------------- Choose an ID, Check if present for atleast 10 frames ---------------------------------#

                objects = self.__tracker.update(r)
                flag = 0
                next_id = 0
                i = 0
                new_coord = []
                next_coord = []
                coord = []

                for (objectID, centroid) in objects.items():
                    if objectID == track_id:
                        flag = 1
                        new_coord = centroid
                    if i == 0:
                        next_id = objectID
                        next_coord = centroid
                        i += 1
                    if objectID in counter:
                        counter[objectID] += 1
                    else:
                        counter[objectID] = 0

                if len(objects.items()) > 0:

                    if flag == 0:
                        track_id = next_id
                        coord = next_coord
                    else:
                        coord = new_coord

                    # --------------------------------- Control LED till 10 frames ---------------------------------#

                    if len(coord) > 0:
                        # If a person exists compute the index of the led to turn on
                        x_pos = coord[0]
                        led_index = math.floor((x_pos * 3.0) / frame_w)
                        animation: LedAnimation = LedAnimation.ANIM_EYE_0
                        if led_index == 0:
                            animation = LedAnimation.ANIM_EYE_0
                        if led_index == 1:
                            animation = LedAnimation.ANIM_EYE_1
                        if led_index == 2:
                            animation = LedAnimation.ANIM_EYE_2
                        # Ask the led manager to play the animation
                        LedController.play_animation(animation)
                        cv2.line(frame, (int(frame_w / 3), 0), (int(frame_w / 4), frame_h), (0, 255, 0), 3)
                        cv2.line(frame, (int(frame_w / 3 * 2), 0), (int(frame_w / 4 * 2), frame_h), (0, 255, 0), 3)
                        cv2.putText(frame, "Led index: %d" % led_index, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (255, 0, 0), 2)  # Draw label text
                        
                    # --------------------------------- Set the handler as true if a person presemt for more than 20 frames ---------------------------------#

                    if counter[track_id] > 20:
                        self.__detection_state = self.__detection_state.on_detection_result(True)

                # All the results have been drawn on the frame, so it's time to display it.
                cv2.imshow('Object detector', frame)

                # Press 'q' to quit
                if cv2.waitKey(1) == ord('q'):
                    break

    def stop(self) -> None:
        self.__alive = False

    def set_detection_handler(self, detection_handler: PeopleDetectionHandler):
        self.__detection_handler = detection_handler
        self.__detection_state.set_detection_handler(detection_handler)
