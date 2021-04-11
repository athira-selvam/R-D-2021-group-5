import rospy
from std_msgs.msg import String
import random

class MusicController:
	music_track = ["track_mozart-the-marriage-of-figaro.wav"]
	music_animation = ["34000:four_four:129"]
	instruments = ["001_1", "001_2","002_1","002_2"]
	tempos =  ["001","002","003","004","005","006"] #e.g. 40 60 80 100 120 140
	active_instrument = []
	active_track = []
	active_tempo = 80 #default
	active_rhythm = "four_four"
	playing_track = False
	pub_speaker = None
	pub_stick = None
	
	def __init__(self):
		self.pub_speaker = rospy.Publisher('audio_track', String, queue_size=10)
		self.pub_stick = rospy.Publisher('animate', String, queue_size=10)
		
	def on_detected_people(self, data):
		#start play random music for 20 seconds
		i = random.randint(0, len(music_track))
		self.playing_track = True
		self.pub_speaker.publish("start_track:" + self.music_track[i])
		self.pub_stick.publish(self.music_animation[i])
		
	def on_code(self,data):
		if code in self.instrument:
			if self.active_instrument == []:#if no active elements at this point start animation indefinite
				self.pub_stick.publish("start:" + self.active_rhythm + ":" + str(self.active_tempo))
				
			instrument = code.split("_")[0]
			track = code.split("_")[1]
			managed = False
			for i in range(len(self.active_instrument)):
				if track==self.active_track[i] and instrument==self.active_instrument[i]:
					#remove the instrument
					self.pub_speaker.publish("stop:" + code)
					self.active_track.pop(i)#remove
					self.active_instrument.pop(i)#remove
					managed = True
				elif instrument==self.active_instrument[i] and track!=self.active_track[i]:
					#change track
					self.pub_speaker.publish("stop:" + self.active_instrument[i] + "_" + self.active_track[i])
					self.pub_speaker.publish("start:" + code + "_" + str(self.active_tempo))
					self.active_track[i] = track#change track
					managed = True
				else:
					pass
			
			if not managed:#no instrument was stopped no instrument was chaghed track => start instrument
				self.pub_speaker.publish("start:" + code + "_" + str(self.active_tempo))
				self.active_track.append(track)
				self.active_instrument.append(instrument)
			
			if self.active_instrument == []:#if no active elements at this point stop animation
				self.pub_stick.publish("stop")
				
		elif code in self.tempos:
			#change tempo
			self.pub_speaker.publish("restart_all:" + code )
			
		else:
			pass

rospy.init_node('music_manager')
mc = MusicController()
rospy.Subscriber("code", String, mc.on_code)
rospy.Subscriber("detected_people", String, mc.on_detected_people)
rospy.spin()
