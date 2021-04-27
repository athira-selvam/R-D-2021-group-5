import time

from tinydb import TinyDB, Query


class DatabaseManager():
    """
    A class representing an object used to interact with a local database, to keep track of visitors interacting with the robot
    """

    __db: TinyDB

    def __init__(self):
        # Initialize the database
        self.__db = TinyDB("./visitors_database.json")
        # Clear the DB at each initialization
        self.__db.drop_tables()

    def visitor_exists(self, visitor_id: str) -> bool:
        """Determines whether the provided visitor has been registered in the database

        Args:
            visitor_id (str): The identifier of the visitor

        Returns:
            bool: True if the visitor is present in the database, False otherwise
        """
        query = Query()
        result = self.__db.search(query.id == visitor_id)
        return len(result) > 0

    def has_visitor_left(self, visitor_id: str) -> bool:
        """Determines whether the provided visitor has left the exhibition

        Args:
            visitor_id (str): The identifier of the visitor

        Returns:
            bool: True if the visitor has left the exhibition, False otherwise
        """
        query = Query()
        result = self.__db.search(
            (query.id == visitor_id))
        return len(result) > 0 and result[0]["exit"] != 0

    def write_visitor_entrance(self, visitor_id: str) -> None:
        """Registers the provided visitor into the database. The entry will contain:
        - their identifier
        - the entrance time (automatically retrieved)
        - an empty field for the exit time

        Args:
            visitor_id (str): The identifier of the visitor
        """
        visitor_data = {"id": visitor_id,
                        "entrance": time.time(), "exit": 0}
        self.__db.insert(visitor_data)

    def write_visitor_exit(self, visitor_id: str) -> None:
        """Register the exit time for the given visitor

        Args:
            visitor_id (str): The identifier of the visitor
        """
        query = Query()
        self.__db.update({"exit": time.time()}, query.id == visitor_id)
