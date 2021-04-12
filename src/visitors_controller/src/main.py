import rospy
from std_msgs.msg import String
from DatabaseManager import DatabaseManager


class VisitorsController():

    __db_manager: DatabaseManager
    __pub: rospy.Publisher

    def __init__(self):
        rospy.init_node("visitors_controller")
        rospy.loginfo("Visitors controller has started")
        rospy.Subscriber("qrcode", String, self.code_handler)

        self.__pub = rospy.Publisher("quiz", String, queue_size=10)
        # Instantiate the local database manager
        self.__db_manager = DatabaseManager()

        rospy.spin()

    def code_handler(self, data):
        """Handles a code received by the QR code reader, published on the /qrcode topic
        Here we assume that detected code can be just visitor ids (printed on tickets)

        Args:
            data (String): The content of the detected code
        """
        # First we extract the content of the message
        visitor_id = data.data
        # Then we ask the database manager whether the user has already entered or not
        if(not self.__db_manager.visitor_exists(visitor_id)):
            self.__db_manager.write_visitor_entrance(visitor_id)
            # TODO: Maybe publish a message saying the visitor has been registered
            # TODO: Publish on a topic to react
            return

        # Here we know that the visitor has already entered the exhibition,
        # hence we check if they have to do the quiz
        if(self.__db_manager.has_visitor_left(visitor_id)):
            # Here the user has already participated in the quiz
            # TODO: React to the error
            return

        # In this last case the user has to participate in the quiz
        self.__db_manager.write_visitor_exit(visitor_id)
        # We send a message on the quiz topic containing the visitor id
        self.__pub.publish(visitor_id)
        # TODO: react and say something


if __name__ == "__main__":
    visitors_controller = VisitorsController()
