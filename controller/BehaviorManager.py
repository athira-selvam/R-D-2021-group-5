from body.LedController import LedController
from body.stick_manager import StickManager
from camera.PeopleDetectionHandler import PeopleDetectionHandler
from camera.QRCodeHandler import QRCodeHandler


class BehaviorManager(QRCodeHandler, PeopleDetectionHandler):
    """
    An abstract class representing an object controlling the behavior of the robot
    """

    _sticks_manager: StickManager
    _led_controller: LedController

    def __init__(self):
        self._sticks_manager = StickManager()
        self._led_controller = LedController()
