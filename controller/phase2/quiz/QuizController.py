import enum
import random
import time
from threading import Thread
from typing import List, Optional

from body.LedController import LedController, LedAnimation
from body.speaker_manager import SpeakerManager
from camera.QRCodeHandler import QRCodeHandler
from controller.phase2.quiz.QuizCompletionHandler import QuizCompletionHandler

QUESTIONS_LIMIT = 2


class QuizAnswer(enum.Enum):
    TRUE = 1,
    FALSE = 2


class QuizQuestion:
    __question: str
    __answer: QuizAnswer
    __audio_file: str  # The name of the audio file for the question
    __explanation_file: Optional[str]

    def __init__(self, question: str, answer: QuizAnswer, audio_file: str, explanation_file: Optional[str] = None):
        self.__question = question
        self.__answer = answer
        self.__audio_file = audio_file
        self.__explanation_file = explanation_file

    def get_question(self) -> str:
        return self.__question

    def get_answer(self) -> QuizAnswer:
        return self.__answer

    def get_audio_file(self) -> str:
        return self.__audio_file

    def get_explanation_file(self) -> Optional[str]:
        return self.__explanation_file


quiz_questions: List[QuizQuestion] = [
    QuizQuestion("test question", QuizAnswer.TRUE, "cucine"),
    QuizQuestion("false?", QuizAnswer.FALSE, "cucine"),
    QuizQuestion("maybe not", QuizAnswer.TRUE, "cucine"),
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
    __to_repeat: bool  # Indicates whether the last question has to be repeated
    __received_answer: Optional[str]

    __speaker: SpeakerManager
    __led_controller: LedController

    __completion_handler: QuizCompletionHandler

    def __init__(self, completion_handler):
        super().__init__()
        self.__alive = True
        self.__received_answer = None
        self.__asked_questions = 0
        self.__to_repeat = False
        self.__picked_questions = []
        self.__speaker = SpeakerManager()
        self.__led_controller = LedController()
        self.__completion_handler = completion_handler

    def __pick_question(self) -> QuizQuestion:

        # First check whether we have to repeat last question
        if self.__to_repeat:
            self.__to_repeat = False
            # If yes just take the last sampled question
            return quiz_questions[self.__picked_questions[len(self.__picked_questions) - 1]]

        # Otherwise generate random number between 0 and the total number of questions
        q_index = random.randint(0, len(quiz_questions) - 1)
        while q_index in self.__picked_questions:
            # And avoid generating indexes already picked
            q_index = random.randint(0, len(quiz_questions) - 1)
        # Eventually append the index to the list of used ones
        self.__picked_questions.append(q_index)
        # And return the question
        return quiz_questions[q_index]

    @staticmethod
    def __parse_answer(raw_answer: str) -> (bool, bool, Optional[QuizAnswer]):
        if raw_answer is None or not raw_answer.startswith("quiz_answer:"):
            # If the code is not a valid answer code, return an error
            return False, False, None
        # Otherwise parse it
        answer_content = raw_answer.split(":")[1]
        if answer_content == "R":
            # Here we need to repeat the question
            return True, True, None
        if answer_content == "T":
            return True, False, QuizAnswer.TRUE
        elif answer_content == "F":
            return True, False, QuizAnswer.FALSE
        else:
            return False, False, None

    def run(self) -> None:
        while self.__alive:
            # Pick the question
            question = self.__pick_question()
            print("The question is: %s" % question.get_question())
            # Ask the question
            self.__speaker.start_audio_track(question.get_audio_file())
            # And wait for the audio track to finish
            time.sleep(self.__speaker.get_track_length(question.get_audio_file()))
            # Show the blinking eye animation
            self.__led_controller.play_animation(LedAnimation.ANIM_IDLE)
            # And then wait for the answer
            while self.__received_answer is None:
                # Sleep until an answer is present
                print("Waiting for answer")
                time.sleep(0.5)

            print("Answer available: ", self.__received_answer)

            # Then parse the answer
            valid, repeat, answer = self.__parse_answer(self.__received_answer)
            # Clear the received answer
            self.__received_answer = None

            if not valid:
                # TODO: React to an invalid answer
                # Show an error animation on the LEDs
                self.__led_controller.play_animation(LedAnimation.ANIM_ERROR)
                # And an audio feedback
                self.__speaker.start_audio_track("invalid_answer")
                time.sleep(self.__speaker.get_track_length("invalid_answer"))
                # And repeat the question
                repeat = True

            # Then check whether the person wants the question to be repeated
            if repeat:
                print("The question will be repeated")
                # If yes we set the flag to True
                self.__to_repeat = True
                # And start a new iteration, without going any further
                continue

            # Then check the answer
            if question.get_answer() == answer:
                print("Answer is right")
                self.__led_controller.play_animation(LedAnimation.ANIM_SUCCESS)
                self.__speaker.start_audio_track("right_answer")
                time.sleep(self.__speaker.get_track_length("right_answer"))
            else:
                print("Answer is wrong")
                self.__led_controller.play_animation(LedAnimation.ANIM_ERROR)
                self.__speaker.start_audio_track("wrong_answer")
                time.sleep(self.__speaker.get_track_length("wrong_answer"))

            # Then if the question as an attached explanation, we say it:
            explanation = question.get_explanation_file()
            if explanation is not None:
                self.__led_controller.play_animation(LedAnimation.ANIM_IDLE)
                self.__speaker.start_audio_track(explanation)
                time.sleep(self.__speaker.get_track_length(explanation))

            # Increase the counter of questions
            self.__asked_questions += 1
            # And stop if we asked all the questions
            if self.__asked_questions >= QUESTIONS_LIMIT:
                print("Reached questions limit, stopping")
                self.__speaker.start_audio_track("quiz_end")
                time.sleep(self.__speaker.get_track_length("quiz_end"))
                self.stop()
        # Here the quiz is over (either because we stopped it or because we reached the last question)
        # Hence we notify the completion handler
        self.__completion_handler.on_quiz_completed()

    def stop(self):
        self.__alive = False

    def handle_code(self, code_content: str):
        super().handle_code(code_content)
        print("QUIZ: received code ", code_content)
        # Update the reference to the received answer
        self.__received_answer = code_content
