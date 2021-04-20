import random
from stick_manager import StickManager
from speaker_manager import SpeakerManager

class MusicController:
	instance = None
	music_track = ["beethoven-fur-elise","four-seasons-vivaldi_autumn","four-seasons-vivaldi-spring","ludwig-van-beethoven-inno-alla-gioia","mozart-the-marriage-of-figaro", "pachelbel-canon"]
	music_animation = [[33000, "four_four", 143],[24000, "four_four", 107],[23000, "four_four", 184],[27000, "four_four", 69],[33000, "four_four", 129],[25000, "four_four", 152]]
	#instruments = ["bassoon", "brass_ensemble","clarinet","piano","strings"]
	#traks = ["1", "2","3","4"]
	#tempos =  ["60","80","100","120"]
	active_instrument = []
	active_track = []
	active_tempo = 80 #default
	active_synch_interval = (60000/80)*4
	active_rhythm = "four_four"
	stick_manager = None
	speaker_manager = None
	
	def __init__(self):
		if self.instance == None:
			MusicController.instance = self
			self.stick_manager = StickManager()
			self.speaker_manager = SpeakerManager.get_instance()
	
	@staticmethod
	def get_instance():
		if MusicController.instance == None:
			MusicController()
		return MusicController.instance
	
	def on_detected_person(self):
		i = random.randint(0, len(self.music_track)-1)
		self.speaker_manager.start_audio_track( self.music_track[i], 0, 0)
		self.stick_manager.start_animation(self.music_animation[i][1], self.music_animation[i][2], self.music_animation[i][0], 0)
		
	def on_code(self,code):
		if code.startswith("i:"):
			code = code[2:] #remove first 2 char
			if self.active_instrument == []:#if no active elements at this point start animation indefinite
				self.stick_manager.start_animation(self.active_rhythm, self.active_tempo, "indefinite", self.active_synch_interval)
			instrument = code.split("-")[0]
			track = code.split("-")[1]
			managed = False
			for i in range(len(self.active_instrument)):
				if track==self.active_track[i] and instrument==self.active_instrument[i]:
					#remove the instrument
					self.speaker_manager.stop_audio_track(code + "-" + str(self.active_tempo), self.active_synch_interval)
					self.active_track.pop(i)#remove
					self.active_instrument.pop(i)#remove
					managed = True
					break
				elif instrument==self.active_instrument[i] and track!=self.active_track[i]:
					#change track
					self.speaker_manager.switch_audio_track([code + "-" + str(self.active_tempo)], 
															[self.active_instrument[i]+"-"+self.active_track[i]+"-"+str(self.active_tempo)], 
															-1,
															self.active_synch_interval)
					self.active_track[i] = track#change track
					managed = True
			
			if not managed:#no instrument was stopped no instrument was chaghed track => start instrument
				self.speaker_manager.start_audio_track(code + "-" + str(self.active_tempo), -1, self.active_synch_interval)
				self.active_track.append(track)
				self.active_instrument.append(instrument)
			
			if self.active_instrument == []:#if no active elements at this point stop animation
				self.stick_manager.stop_animation()
				print("stopping all animations")
		elif code.startswith("t:"):
			code = code[2:] #remove first 2 char
			#change tempo
			self.speaker_manager.switch_audio_track([i+"-"+t+"-"+str(code) for i in self.active_instrument for t in self.active_track], 
													[i+"-"+t+"-"+str(self.active_tempo) for i in self.active_instrument for t in 														self.active_track], 
													-1,
													self.active_synch_interval)
			self.stick_manager.change_tempo(code)
			self.active_tempo = int(code)
			active_synch_interval = (60000/self.active_tempo)*4

if __name__ == "__main__":
	mc = MusicController.get_instance()
	inpt = ""
	while(inpt != "q"):
		inpt = input("Type p for PersonDetected, i:instrument-track for Instrument, t:tempo for tempo: ")
		if inpt == "p":
			mc.on_detected_person()
		else:
			mc.on_code(inpt)
	print("exit")
