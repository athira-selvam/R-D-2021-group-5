import rospy
from std_msgs.msg import String
import multiprocessing
from playsound import playsound

class SpeakerManager:
	#an Intrument Track name will be composed as follow:
	#instrumentCode_track_tempo.mp3
	#e.g. violin_3_120.mp3
	
	#a full Track name will be composed as follow:
	#track_nameofthetrack.mp3
	music_processes = []
	music_processes_name = []
	tempos = [40, 60, 80, 100, 120, 140]#possible tempos
	
	def __init__(self):
		pass
	
	def start_process(self, name, extension):
		self.music_processes_name.append(name)
		process = multiprocessing.Process(target=playsound, args=(name + extension ,))
		self.music_process.append(process)
		process.start()
	
	def stop_process(self, name):
		#stop the process
		for i in range(len(self.music_processes_name)):
			if self.music_processes_name[i].startswith(name):
				self.music_process[i].terminate()
				self.music_process.pop(i)
				self.music_processes_name.pop(i)
	
	def on_audio_track(self, data):
		#understand if it is a track or instrument
		if data.split(":")[0] == "start_track" :
			#create the process
			self.start_process(data.split(":")[1], "")#no extension
			
		elif data.split(":")[0] == "stop_track":
			#stop the process
			self.stop_process(data.split(":")[1], "")
			
		elif data.split(":")[0] == "start" :
			#create the process
			self.start_process(data.split(":")[1], ".mp3")#no extension
		
		elif data.split(":")[0] == "stop" :
			#stop the process
			self.stop_process(data.split(":")[1], "")
		
		elif data.split(":")[0] == "restart_all":
			#copy array
			local = self.music_processes_name
			for i in range(len(local)):
				if not local[i].startswith("track"):
					#compose the new name
					parts = local[i].split("_")
					name = str(parts[0]) + "_" + str(parts[1]) + "_" + data.split(":")[1] #old intrument, new tempo
					self.stop_process(local[i])
					self.start_process(name,".mp3")
				
	def on_talk(self, data):
		#not yet implemented
		pass

rospy.init_node('speaker_manager')
sm = SpeakerManager()
rospy.Subscriber("audio_track", String, sm.on_audio_track)
rospy.Subscriber("talk", String, sm.on_talk)
rospy.spin()
