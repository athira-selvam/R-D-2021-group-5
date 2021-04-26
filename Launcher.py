import sys

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

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        if not code_content.startswith("phase:"):
            print("This is not a valid phase-choice code")

        phase_code = code_content.split(":")[1]
        if phase_code != "1" and phase_code != "2":
            print("Phase is not valid")
            sys.exit(0)
        if phase_code == "1":
            # Start phase 1
            self.__behavior_manager = MusicController()
            print("Launched phase 1")
        elif phase_code == "2":
            self.__behavior_manager = VisitorsController()
            print("Launched phase 2")
        self.__camera_controller.subscribe_to_qrcode(self.__behavior_manager)
        self.__camera_controller.subscribe_to_people_detection(self.__behavior_manager)


if __name__ == "__main__":
    # Instantiate the launcher
    launcher = Launcher()
