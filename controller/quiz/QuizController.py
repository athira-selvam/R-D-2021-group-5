import enum
import random
import time
from threading import Thread
from typing import List, Optional

from body.speaker_manager import SpeakerManager
from camera.QRCodeHandler import QRCodeHandler

QUESTIONS_LIMIT = 2


class QuizAnswer(enum.Enum):
    TRUE = 1,
    FALSE = 2


class QuizQuestion:
    __question: str
    __answer: QuizAnswer
    __audio_file: str  # The name of the audio file for the question

    def __init__(self, question: str, answer: QuizAnswer, audio_file: str):
        self.__question = question
        self.__answer = answer
        self.__audio_file = audio_file

    def get_question(self) -> str:
        return self.__question

    def get_answer(self) -> QuizAnswer:
        return self.__answer

    def get_audio_file(self) -> str:
        return self.__audio_file


quiz_questions: List[QuizQuestion] = [
    QuizQuestion("test question", QuizAnswer.TRUE, "sample"),
    QuizQuestion("false?", QuizAnswer.FALSE, "sample"),
    QuizQuestion("maybe not", QuizAnswer.TRUE, "sample"),
]


# TODO: Maybe here we can set a duration for each question, so we know how much to wait?


class QuizController(Thread, QRCodeHandler):
    """
    A class, implementing the QR Code Handler interface, representing an object that can manage the quiz for one visitor
    """

    # Here we keep the indexes of the questions we selected, to avoid repeating them
    __picked_questions: List[int]
    __alive: bool

    __asked_questions: int  # A counter of the questions asked so far

    __received_answer: Optional[str]

    __speaker: SpeakerManager

    def __init__(self):
        super().__init__()
        self.__alive = True
        self.__received_answer = None
        self.__asked_questions = 0
        self.__picked_questions = []
        self.__speaker = SpeakerManager()

    def __pick_question(self) -> QuizQuestion:
        # generate random number between 0 and the total number of questions
        q_index = random.randint(0, len(quiz_questions) - 1)
        while q_index in self.__picked_questions:
            # And avoid generating indexes already picked
            q_index = random.randint(0, len(quiz_questions) - 1)
        # Eventually append the index to the list of used ones
        self.__picked_questions.append(q_index)
        # And return the question
        return quiz_questions[q_index]

    @staticmethod
    def __parse_answer(raw_answer: str) -> (bool, Optional[QuizAnswer]):
        if raw_answer is None or not raw_answer.startswith("quiz_answer:"):
            # If the code is not a valid answer code, return an error
            return False, None
        # Otherwise parse it
        answer_content = raw_answer.split(":")[1]
        if answer_content == "T":
            return True, QuizAnswer.TRUE
        elif answer_content == "F":
            return True, QuizAnswer.FALSE
        else:
            return False, None

    def run(self) -> None:
        while self.__alive:
            # Pick the question
            question = self.__pick_question()
            print("The question is: %s" % question.get_question())
            # Ask the question
            self.__speaker.start_audio_track(question.get_audio_file(), 0, 0)
            # And then wait for the answer
            while self.__received_answer is None:
                # Sleep until an answer is present
                print("Waiting for answer")
                time.sleep(0.5)

            print("Answer available: ", self.__received_answer)

            # Then parse the answer
            valid, answer = self.__parse_answer(self.__received_answer)
            # Clear the received answer
            self.__received_answer = None

            # Then check the answer
            if question.get_answer() == answer:
                print("Answer is right")
                self.__speaker.start_audio_track("right_answer", 0, 0)
                pass
            else:
                print("Answer is wrong")
                self.__speaker.start_audio_track("wrong_answer", 0, 0)
                pass

            # Increase the counter of questions
            self.__asked_questions += 1
            # And stop if we asked all the questions
            if self.__asked_questions >= QUESTIONS_LIMIT:
                print("Reached questions limit, stopping")
                self.stop()
                # TODO: Find a way to notify VisitorsManager that quiz has finished
            # Then wait a bit before picking the next question
            time.sleep(2)  # TODO: adjust this time interval

    def stop(self):
        self.__alive = False

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        # Update the reference to the received answer
        self.__received_answer = code_content
