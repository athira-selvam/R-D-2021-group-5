from typing import Optional

from controller.quiz.QuizController import QuizController
from controller.BehaviorManager import BehaviorManager
from controller.phase2.DatabaseManager import DatabaseManager


class VisitorsController(BehaviorManager):
    __db_manager: DatabaseManager
    # The object controlling the quiz workflow, it can be None if no person is playing
    __quiz_controller: Optional[QuizController]

    def __init__(self):
        super().__init__()
        # Instantiate the local database manager
        self.__db_manager = DatabaseManager()

    def handle_person(self, is_person_present: bool) -> None:
        super().handle_person(is_person_present)

    def handle_code(self, code_content: str):
        super().handle_code(code_content)

        # First we extract the content of the message
        visitor_id = code_content
        # Then we ask the database manager whether the user has already entered or not
        if not self.__db_manager.visitor_exists(visitor_id):
            self.__db_manager.write_visitor_entrance(visitor_id)
            # TODO: Maybe publish a message saying the visitor has been registered
            # TODO: Publish on a topic to react
            return

        # Here we know that the visitor has already entered the exhibition,
        # hence we check if they have to do the quiz
        if self.__db_manager.has_visitor_left(visitor_id):
            # Here the user has already participated in the quiz
            # TODO: React to the error
            return

        # In this last case the user has to participate in the quiz
        self.__db_manager.write_visitor_exit(visitor_id)
        # We send a message on the quiz topic containing the visitor id
        # TODO: Instantiate quiz controller and enter in quiz mode
        self.__quiz_controller = QuizController()

        # TODO: Set the state

        # TODO: react and say something

    def get_quiz_controller(self) -> QuizController:
        return self.__quiz_controller
