import sys

from body.LedController import LedController ,LedAnimation
from body.speaker_manager import SpeakerManager
from camera.CameraController import CameraController
from camera.QRCodeHandler import QRCodeHandler
from controller.BehaviorManager import BehaviorManager
from controller.phase1.music_controller import MusicController
from controller.phase2.VisitorsController import VisitorsController


class Launcher(QRCodeHandler):
    __camera_controller: CameraController
    __behavior_manager: BehaviorManager

    def __init__(self):
        # First we initialize the camera controller
        self.__camera_controller = CameraController()
        self.__camera_controller.start()
        # And register ourselves as the code handlers
        self.__camera_controller.subscribe_to_qrcode(self)

    def handle_code(self, phase_code: str):
        super().handle_code(phase_code)

        if phase_code != "inside" and phase_code != "outside":
            print("Phase is not valid")
            return
        if phase_code == "outside":
            # Start phase 1
            SpeakerManager().start_track_and_wait("outside")
            self.__behavior_manager = MusicController()
            print("Launched inside phase")
        elif phase_code == "inside":
            SpeakerManager().start_track_and_wait("inside")
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
    lc.start()
    lc.play_animation(LedAnimation.ANIM_IDLE)
