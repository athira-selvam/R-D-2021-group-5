class DatabaseManager():
    """A class representing an object used to interact with a local database, to keep track of visitors interacting with the robot
    """

    def __init__(self):
        pass

    def visitor_exists(self, visitor_id: str) -> bool:
        return True

    def has_visitor_left(self, visitor_id: str) -> bool:
        return False
        
    def write_visitor_entrance(self, visitor_id: str) -> None:
        pass

    def write_visitor_exit(self, visitor_id: str) -> None:
        pass

