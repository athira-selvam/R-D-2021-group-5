from typing import Optional, Callable

from controller.BehaviorManager import BehaviorManager
from controller.phase2.DatabaseManager import DatabaseManager
from controller.phase2.quiz.QuizCompletionHandler import QuizCompletionHandler
from controller.phase2.quiz.QuizController import QuizController


class VisitorsController(BehaviorManager, QuizCompletionHandler):
    __db_manager: DatabaseManager
    # The object controlling the quiz workflow, it can be None if no person is playing
    __quiz_controller: Optional[QuizController]
    # Function to map the incoming code to the right handler method
    __state_code_handler: Callable[[str], None]

    def __create_handler_function(self, is_quiz: bool) -> Callable[[str], None]:
        if not is_quiz:
            return lambda code: self.__handle_visitor_code(code)
        else:
            return lambda code: self.__quiz_controller.handle_code(code)

    def __init__(self):
        super().__init__()
        # Instantiate the local database manager
        self.__db_manager = DatabaseManager()
        # And initialize the idle state
        self.__state_code_handler = self.__create_handler_function(False)

    def handle_person(self, is_person_present: bool) -> None:
        super().handle_person(is_person_present)

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        self.__state_code_handler(code_content)

    def __handle_visitor_code(self, code_content: str):
        print("Received code %s" % code_content)

        if not code_content.startswith("visitor:"):
            print("Invalid visitor code")
            return

        # First we extract the content of the message
        visitor_id = code_content.split(":")[1]
        print("Handling visitor %s" % visitor_id)
        # Then we ask the database manager whether the user has already entered or not
        if not self.__db_manager.visitor_exists(visitor_id):
            self.__db_manager.write_visitor_entrance(visitor_id)
            # TODO: Maybe publish a message saying the visitor has been registered
            # TODO: Publish on a topic to react
            print("Written visitor %s entrance" % visitor_id)
            return

        # Here we know that the visitor has already entered the exhibition,
        # hence we check if they have to do the quiz
        if self.__db_manager.has_visitor_left(visitor_id):
            # Here the user has already participated in the quiz
            # TODO: React to the error
            print("Visitor %s already left exhibition" % visitor_id)
            return

        # In this last case the user has to participate in the quiz
        self.__db_manager.write_visitor_exit(visitor_id)
        print("Written visitor %s exit" % visitor_id)
        # We therefore instantiate the quiz controller, passing ourselves as the completion handler
        self.__quiz_controller = QuizController(self)
        self.__quiz_controller.start()
        print("Started quiz controller")
        # And enter in quiz state
        self.__state_code_handler = self.__create_handler_function(True)
        # TODO: react and say something

    def get_quiz_controller(self) -> QuizController:
        return self.__quiz_controller

    def on_quiz_completed(self) -> None:
        print("Received quiz completion event")
        # Here we just update the code handler to be in idle state
        self.__state_code_handler = self.__create_handler_function(False)
