import rospy
from std_msgs.msg import String


class CodeFilter():
    """
    A class used to filter messages published on the /barcode topic,
    to avoid the same QR code being broadcasted multiple times
    """

    def __init__(self):
        # Initialize the filter
        self.last_code = None
        # Create the publisher for the filtered topic
        self.pub = rospy.Publisher("qrcode", String, queue_size=10)
        # Initialize the node
        rospy.init_node("code_filter")
        # Subscribe to topic
        rospy.Subscriber("barcode", String, self.handle_code)
        rospy.spin()

    def handle_code(self, data):
        received_code = data.data
        # We check whether the received code is the same as the last one
        if self.last_code is not None and self.last_code == received_code:
            # If yes we just stop
            rospy.loginfo("Blocked code %s", received_code)
            return
        # Otherwise we save it as the last emitted code
        self.last_code = received_code
        # And publish it on the filtered topic
        self.pub.publish(received_code)

        rospy.loginfo(rospy.get_caller_id() +
                      " Received code %s", received_code)


if __name__ == "__main__":
    code_filter = CodeFilter()
