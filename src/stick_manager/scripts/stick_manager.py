from stick import Stick
import rospy
from std_msgs.msg import String
import matplotlib

class StickManager:
	left_stick = None
	right_stick = None
	
	def __init__(self):
		self.left_stick = Stick(True, 80, [0, 1])
		self.right_stick = Stick(False, 80, [2, 3])
	
	#message of type: rhythm,time,initial_tempo (e.g., four_four,3000,80)
	def on_animation(data):
		animation = data.split(",")
		self.left_stick.animate(rythm=animation[0], duration=animation[1], initial_tempo==animation[2])
		self.right.animate(rythm=animation[0], duration=animation[1], initial_tempo==animation[2])
	
	#message of type: tempo (e.g., 80)
	def on_tempo(data):
		self.left_stick.new_tempo(data)
		self.right_stick.new_tempo(data)
	
	#message of type: style (e.g., four_four)
	def on_style(data):
		self.left_stick.set_rhythm(data)
		self.right_stick.set_rhythm(data)

rospy.init_node('stick_manager')
rospy.Subscriber("tempo", String, callback)
rospy.Subscriber("style", String, callback)
rospy.Subscriber("animation", String, callback)
rospy.spin()

	
