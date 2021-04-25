import importlib.util
import os
from threading import Thread

import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter

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

    __detection_handler: PeopleDetectionHandler
    __tracker: CentroidTracker

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
        counter = {}
        rot = 0

        
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
                frame_w = frame.shape[1]
                rects = []
                r = []
                val = []
                track_id = 0


                for i in range(len(scores)):
                    if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
                       
                        object_name = self.__labels[int(classes[i])]
                        if object_name != "person":
                            continue
                        
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

                        val = np.array(rects)
                        r.append(val.astype("int"))

 #---------------------------------Head random rotation, rotate to 90 if any people detected---------------------------------#               
                
                if(r):
                    __head.rotate(90)
                else:
                    if(rot = 0):
                        __head.rotate(0)
                    else:
                        __head.rotate(180)
                        
                if(rot == 0):
                    rot = 180
                else:
                    rot = 0

 #--------------------------------- Choose an ID, Check if present for atleast 10 frames ---------------------------------# 
                
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
                    if(objectID in counter):
                        counter[objectID] +=1
                    else:
                        counter[objectID] = 0
                    
                if flag == 0:
                    track_id = next_id
                    coord = next_coord
                else:
                    coord = new_coord

 #--------------------------------- Control LED till 10 frames ---------------------------------# 

                if(coord):
                    led_pos = coord[0]
                
                index = math.floor((led_pos*4)/frame_w)

                #call led with position and led position + 4 -> (2 leds)    



 #--------------------------------- Set the handler as true if a person presemt for more than 20 frames ---------------------------------#             

                if(counter[track_id]>20):
                    print("setting people detection handler to true")
                    self.__detection_handler.handle_person(is_person_present: True)
                
                #callmusic
                

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

    def set_detection_handler(self, detection_handler: PeopleDetectionHandler):
        self.__detection_handler = detection_handler
