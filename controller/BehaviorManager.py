from camera.PeopleDetectionHandler import PeopleDetectionHandler
from camera.QRCodeHandler import QRCodeHandler
from controller.LedController import LedController
from controller.SticksController import SticksController


class BehaviorManager(QRCodeHandler, PeopleDetectionHandler):
    """
    An abstract class representing an object controlling the behavior of the robot
    """

    _sticks_controller: SticksController
    _led_controller: LedController

    def __init__(self):
        self._sticks_controller = SticksController()
        self._led_controller = LedController()
