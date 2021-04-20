from pygame import mixer
import time
from math import floor, ceil

class SpeakerManager:
	instance = None
	sound = [None for i in range(12)]
	track_name = [None for i in range(12)]
	channel = [0 for i in range(12)]
	sounds_path = "../Sounds/"
	sounds_extension = ".wav"
	
	@staticmethod
	def get_instance():
		if SpeakerManager.instance == None:
			SpeakerManager()
		return SpeakerManager.instance
	
	def __init__(self):
		if self.instance == None:
			SpeakerManager.instance = self
			mixer.pre_init(44100,-16,12, 1024)
			mixer.init()
	
	def get_id(self,track_name):
		for i in range(len(self.track_name)):
			if track_name == self.track_name[i]:
				return i
		#not found
		return -1
	
	def get_free_channel(self):
		for i in range(len(self.channel)):
			if self.channel[i]==0:
				self.channel[i]=1 #set busy
				return i
		#else channels are full
		return -1
	
	def synchronize(self, synch_interval):
		if synch_interval == 0:
			return
		next_second = ceil(time.time())*1000
		next_tick = next_second + (next_second%synch_interval)
		print("synch at: " + str(next_tick) + ", synch_interval: " + str(synch_interval))
		wait_for = (next_tick - time.time()*1000)/1000
		if wait_for > 0:
			time.sleep(wait_for)
	
	def start_audio_track(self, name, loops, synch_interval):
		"""Play the track named = name and when the time is multiple of synch_interval (in milliseconds)"""
		name = self.sounds_path + name + self.sounds_extension
		index = self.get_free_channel()
		if 0 <= index <= 12:
			#load file
			self.track_name[index] = name
			self.sound[index] = mixer.Sound(name)
			#synchronization
			self.synchronize(synch_interval)
			mixer.Channel(index).play(Sound=self.sound[index], loops=loops)
			print("started at : " + str(time.time()*1000))
		else:
			#channel are full
			print("All channels are full try again")
	
	def switch_audio_track(self, start_names, stop_names, loops, synch_interval):
		"""Switch all the track inside stop_names to the relative one inside start_names and perform the operations 
		when the time is multiple of synch_interval (in milliseconds)"""
		#synchronization
		self.synchronize(synch_interval)
		for i in range(len(start_names)):
			start_names[i] = self.sounds_path + start_names[i] + self.sounds_extension
			stop_names[i] = self.sounds_path + stop_names[i] + self.sounds_extension
			index = self.get_id(stop_names[i])
			if 0 <= index <= 12:
				#stop
				mixer.Channel(index).stop()
				#load file
				self.track_name[index] = start_names[i]
				self.sound[index] = mixer.Sound(start_names[i])
				mixer.Channel(index).play(Sound=self.sound[index], loops=loops)
				print("switched " + str(i) + "-th at : " + str(time.time()*1000))
			else:
				#Stop_name not found
				print("Stop_name track not found")
	
	def stop_audio_track(self, name, synch_interval):
		name = self.sounds_path + name + self.sounds_extension
		#search the music
		id = self.get_id(name)
		if 0 <= id <= 12:
			#stop if found
			self.channel[id]=0 #free resource
			#synchronization
			self.synchronize(synch_interval)
			mixer.Channel(id).stop()
			print("stopped at : " + str(time.time()*1000))
	
	def pause_audio_track(self, name, synch_interval):
		name = self.sounds_path + name + self.sounds_extension
		#search the music
		id = self.get_id(name)
		if 0 <= id <= 12:
			#pause if found
			#synchronization
			self.synchronize(synch_interval)
			mixer.Channel(id).pause()
			print("paused at : " + str(time.time()*1000))
	
	def unpause_audio_track(self, name, synch_interval):
		name = self.sounds_path + name + self.sounds_extension
		#search the music
		id = self.get_id(name)
		if 0 <= id <= 12:
			#pause if found
			#synchronization
			self.synchronize(synch_interval)
			mixer.Channel(id).unpause()
			print("unpaused at : " + str(time.time()*1000))
