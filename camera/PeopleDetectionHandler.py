from abc import ABC


class PeopleDetectionHandler(ABC):
    def handle_person(self, is_person_present: bool) -> None:
        """
        Reacts to the result of the people detection process
        :param is_person_present: True if a person is detected, False otherwise
        """
        pass
