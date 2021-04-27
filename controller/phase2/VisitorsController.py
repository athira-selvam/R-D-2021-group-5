import multiprocessing
import time
from abc import ABC, abstractmethod
from threading import Thread
from typing import Optional, Callable

from body.speaker_manager import SpeakerManager
from controller.BehaviorManager import BehaviorManager
from controller.phase2.DatabaseManager import DatabaseManager
from controller.phase2.quiz.QuizCompletionHandler import QuizCompletionHandler
from controller.phase2.quiz.QuizController import QuizController


class PeopleDetectionState(ABC):
    _output: bool

    def __init__(self):
        self._output = False

    @abstractmethod
    def on_result(self, person_detected: bool):
        pass

    @abstractmethod
    def should_react_to_person(self) -> bool:
        pass

    def is_output_enabled(self):
        return not self._output

    def consume_output(self):
        self._output = True


class BusyWithFriend(PeopleDetectionState):

    def on_result(self, person_detected: bool):
        return self

    def should_react_to_person(self) -> bool:
        return False


class PersonDetectedState(PeopleDetectionState):

    def on_result(self, person_detected: bool) -> PeopleDetectionState:
        if person_detected:
            return self
        else:
            return IdleState()

    def should_react_to_person(self):
        return False


class IdleState(PeopleDetectionState):

    def on_result(self, person_detected: bool) -> PeopleDetectionState:
        if person_detected:
            return PersonDetectedState()
        else:
            return self

    def should_react_to_person(self) -> bool:
        return True


class VisitorsController(BehaviorManager, QuizCompletionHandler):
    __db_manager: DatabaseManager
    # The object controlling the quiz workflow, it can be None if no person is playing
    __quiz_controller: Optional[QuizController]
    # Function to map the incoming code to the right handler method
    __state_code_handler: Callable[[str], None]

    __detection_state: PeopleDetectionState

    __speaker_manager: SpeakerManager

    __interaction_code: Optional[str]

    def __create_handler_function(self, is_quiz: bool) -> Callable[[str], None]:
        if not is_quiz:
            return lambda code: Thread(target=self.__handle_visitor_code, args=[code]).start()
        else:
            return lambda code: self.__quiz_controller.handle_code(code)

    def __init__(self):
        super().__init__()
        # Instantiate the local database manager
        self.__db_manager = DatabaseManager()
        # And initialize the idle state
        self.__state_code_handler = self.__create_handler_function(False)
        self.__detection_state = IdleState()

        self.__interaction_code = None

        self.__speaker_manager = SpeakerManager()

    def __greet_detected_person(self):
        self.__speaker_manager.start_track_and_wait("welcometicket")

    def handle_person(self, is_person_present: bool) -> None:
        super().handle_person(is_person_present)
        # If there is a person and we should react to it, we react
        if self.__detection_state.should_react_to_person() and is_person_present:
            # If we didn't greet a person before, we do it and consume the output
            if self.__detection_state.is_output_enabled():
                self.__detection_state.consume_output()
                # We run the greeting code in a different thread not to block everything
                Thread(target=self.__greet_detected_person).start()
        # eventually update the state with the result
        self.__detection_state = self.__detection_state.on_result(is_person_present)

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        self.__state_code_handler(code_content)

    def __handle_visitor_code(self, code_content: str):
        print("Received code %s" % code_content)

        if not code_content.startswith("user"):
            print("Invalid visitor code")
            return

        # First we extract the content of the message
        visitor_id = code_content
        # Save the old detection state
        old_detection_state = self.__detection_state
        self.__detection_state = BusyWithFriend()
        print("Handling visitor %s" % visitor_id)
        # Then we ask the database manager whether the user has already entered or not
        if not self.__db_manager.visitor_exists(visitor_id):
            self.__db_manager.write_visitor_entrance(visitor_id)
            # Greet the visitor
            self.__speaker_manager.start_track_and_wait("afterticket")
            print("Written visitor %s entrance" % visitor_id)
            self.__detection_state = old_detection_state
            return

        # Here we know that the visitor has already entered the exhibition,
        # hence we check if they have to do the quiz
        if self.__db_manager.has_visitor_left(visitor_id):
            # Here the user has already participated in the quiz
            # TODO: React to the error
            print("Visitor %s already left exhibition" % visitor_id)
            self.__detection_state = old_detection_state
            return

        # In this last case the user has to participate in the quiz
        self.__db_manager.write_visitor_exit(visitor_id)
        print("Written visitor %s exit" % visitor_id)

        # Here we introduce the visitor to the quiz:
        self.__speaker_manager.start_track_and_wait("quizintro")

        # We create a new handler, to set the interaction code
        self.__state_code_handler = lambda code: self.__handle_interaction_code(code)

        # Then we should wait for the code
        interaction_wait_start = time.time()
        while self.__interaction_code is None:
            time.sleep(0.5)
            interaction_wait_counter = time.time()
            # If the user is idle for more than 5 seconds, just ignore it
            if interaction_wait_counter - interaction_wait_start > 20:
                break
        # Change the code handler back
        self.__state_code_handler = self.__create_handler_function(False)

        # If no reply is available or if the reply is negative, stop there
        if self.__interaction_code is None or self.__interaction_code != "yes":
            self.__detection_state = old_detection_state
            return

        self.__interaction_code = None

        # We therefore instantiate the quiz controller, passing ourselves as the completion handler
        self.__quiz_controller = QuizController(self)
        self.__quiz_controller.start()
        print("Started quiz controller")
        # And enter in quiz state
        self.__state_code_handler = self.__create_handler_function(True)

    def __handle_interaction_code(self, code):
        self.__interaction_code = code

    def get_quiz_controller(self) -> QuizController:
        return self.__quiz_controller

    def on_quiz_completed(self) -> None:
        print("Received quiz completion event")
        # Here we just update the code handler to be in idle state
        self.__state_code_handler = self.__create_handler_function(False)
        self.__detection_state = PersonDetectedState()
