import os
import sys

from body.LedController import LedController
from body.speaker_manager import SpeakerManager
from camera.CameraController import CameraController
from camera.QRCodeHandler import QRCodeHandler
from controller.BehaviorManager import BehaviorManager
from controller.phase1.music_controller import MusicController
from controller.phase2.VisitorsController import VisitorsController
from controller.phase2.quiz.QuizController import QuestionFactory


class Launcher(QRCodeHandler):
    __camera_controller: CameraController
    __behavior_manager: BehaviorManager

    def __init__(self):
        # First we initialize the camera controller
        self.__camera_controller = CameraController()
        self.__camera_controller.start()
        # And register ourselves as the code handlers
        self.__camera_controller.subscribe_to_qrcode(self)

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        if not code_content.startswith("phase:"):
            print("This is not a valid phase-choice code")

        phase_code = code_content.split(":")[1]
        if phase_code != "inside" and phase_code != "outside":
            print("Phase is not valid")
            sys.exit(0)
        if phase_code == "inside":
            # Start phase 1
            SpeakerManager.start_track_and_wait("inside")
            self.__behavior_manager = MusicController()
            print("Launched inside phase")
        elif phase_code == "outside":
            SpeakerManager.start_track_and_wait("outside")
            self.__behavior_manager = VisitorsController()
            print("Launched outside phase")
        self.__camera_controller.subscribe_to_qrcode(self.__behavior_manager)
        self.__camera_controller.subscribe_to_people_detection(self.__behavior_manager)


if __name__ == "__main__":
    # Instantiate the launcher
    launcher = Launcher()

    SpeakerManager().start_track_and_wait("requestmode")

    # And initialize the led controller
    lc = LedController()
    # lc.start()
    launcher.handle_code("phase:1")
