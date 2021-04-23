import sys
from typing import Optional

from camera.CameraController import CameraController
from controller.BehaviorManager import BehaviorManager
from controller.phase1.music_controller import MusicController
from controller.phase2.VisitorsController import VisitorsController


class Launcher(BehaviorManager):

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        if not code_content.startswith("phase:"):
            print("This is not a valid phase-choice code")

        phase_code = code_content.split(":")[1]
        if phase_code != "1" and phase_code != "2":
            print("Phase is not valid")
            sys.exit(0)
        behavior_manager: Optional[BehaviorManager] = None
        if phase_code == "1":
            # Start phase 1
            behavior_manager = MusicController()
            print("Launched phase 1")
        elif phase_code == "2":
            behavior_manager = VisitorsController()
            print("Launched phase 2")


if __name__ == "__main__":
    # Instantiate the Camera Controller, since we need to read QR codes
    camera_controller = CameraController()
    camera_controller.start()
    # Then register
    launcher = Launcher()
    camera_controller.subscribe_to_qrcode(launcher)
