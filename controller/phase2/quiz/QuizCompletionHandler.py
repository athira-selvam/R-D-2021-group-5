from abc import ABC, abstractmethod


class QuizCompletionHandler(ABC):
    """
    An abstract class representing an object that reacts to the end of a quiz,
    exposing a method to be called once the quiz is over
    """

    @abstractmethod
    def on_quiz_completed(self) -> None:
        pass
