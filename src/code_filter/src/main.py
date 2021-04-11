import rospy
from std_msgs.msg import String
import time

TIME_THRESHOLD = 30  # 30 seconds before a code is detected twice


class CodeFilter():
    """
    A class used to filter messages published on the /barcode topic,
    to avoid the same QR code being broadcasted multiple times
    """

    __last_code: str
    __last_time: time.time

    def __init__(self):
        # Initialize the filter
        self.__last_code = None
        self.__last_time = time.time()
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
        if self.__last_code is not None and self.__last_code == received_code:
            # Then check whether the time difference is above the threshold
            now = time.time()
            time_diff = now-self.__last_time
            if(time_diff <= TIME_THRESHOLD):
                # If yes we update the last time and just stop
                self.__last_time = now
                rospy.loginfo("Blocked code %s", received_code)

                return
        # Otherwise we save it as the last emitted code
        self.__last_code = received_code
        self.__last_time = time.time()
        # And publish it on the filtered topic
        self.pub.publish(received_code)

        rospy.loginfo(rospy.get_caller_id() +
                      " Received code %s", received_code)


if __name__ == "__main__":
    code_filter = CodeFilter()
