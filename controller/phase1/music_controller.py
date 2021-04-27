import multiprocessing
import random
import time
from abc import ABC, abstractmethod
from typing import Optional

from body.speaker_manager import SpeakerManager
from body.stick_manager import StickManager
from controller.BehaviorManager import BehaviorManager
from body.LedController import LedController, LedAnimation
from utils.Singleton import Singleton


class MusicControllerState(ABC):
    _output_done: bool

    def __init__(self):
        self._output_done = False

    @abstractmethod
    def is_people_detection_enabled(self) -> bool:
        pass

    @abstractmethod
    def should_handle_person_exit(self) -> bool:
        pass


class MusicPlayingState(MusicControllerState):

    def is_people_detection_enabled(self) -> bool:
        return False

    def should_handle_person_exit(self) -> bool:
        return False


class PersonGreetingState(MusicControllerState):

    def is_people_detection_enabled(self) -> bool:
        return False

    def should_handle_person_exit(self) -> bool:
        return True


class IdleState(MusicControllerState):

    def is_people_detection_enabled(self) -> bool:
        return True

    def should_handle_person_exit(self) -> bool:
        return False


class MusicController(Singleton, BehaviorManager):
    music_track = ["beethoven-fur-elise", "four-seasons-vivaldi_autumn", "four-seasons-vivaldi-spring",
                   "ludwig-van-beethoven-inno-alla-gioia", "mozart-the-marriage-of-figaro", "pachelbel-canon"]
    music_animation = [[33000, "four_four", 143], [24000, "four_four", 107], [23000, "four_four", 184],
                       [27000, "four_four", 69], [33000, "four_four", 129], [25000, "four_four", 152]]
    # instruments = ["bassoon", "brass_ensemble","clarinet","piano","strings"]
    # tracks = ["1", "2","3","4"]
    # tempos =  ["60","80","100","120"]
    active_instrument = []
    active_track = []
    active_tempo = 80  # default
    active_synch_interval = (60000 / 80) * 4
    active_rhythm = "four_four"
    stick_manager = None
    speaker_manager: SpeakerManager = None
    led_controller: LedController

    __music_player_thread: Optional[multiprocessing.Process]

    state: MusicControllerState

    def __init__(self):
        super().__init__()
        self.stick_manager = StickManager()
        self.speaker_manager = SpeakerManager()
        self.state = IdleState()
        self.__music_player_thread = None
        self.led_controller = LedController()

    def __play_greeting_music(self):
        i = random.randint(0, len(self.music_track) - 1)
        self.led_controller.play_animation(LedAnimation.ANIM_MUSIC, track=i)
        track = self.speaker_manager.start_audio_track(self.music_track[i], 0, 0)
        self.stick_manager.start_animation(self.music_animation[i][1], self.music_animation[i][2],
                                           self.music_animation[i][0], 0)
        time.sleep(self.speaker_manager.get_track_length(track))
        # After say the after music speech
        self.speaker_manager.start_track_and_wait("instrumentcard")

    def handle_person(self, is_person_present: bool) -> None:
        if self.state.is_people_detection_enabled and is_person_present:
            print("Entering people detected branch")
            self.state = PersonGreetingState()
            # If the music playing process was already running we kill them, and set up a new one
            if self.__music_player_thread is not None and self.__music_player_thread.is_alive():
                # We also gracefully stop the sticks manager
                self.stick_manager.stop_animation()
                self.__music_player_thread.kill()
            self.__music_player_thread = multiprocessing.Process(target=self.__play_greeting_music)
            self.__music_player_thread.start()
        elif self.state.is_people_detection_enabled() and self.state.should_handle_person_exit() and not is_person_present:
            print("Reacting to person not found")
            if self.__music_player_thread is not None and self.__music_player_thread.is_alive():
                self.__music_player_thread.kill()
            self.stick_manager.stop_animation()
            self.state = IdleState()

    def handle_code(self, code):
        super().handle_code(code)
        if code.startswith("i:"):
            code = code[2:]  # remove first 2 char
            if self.active_instrument == []:  # if no active elements at this point start animation indefinite
                self.stick_manager.start_animation(self.active_rhythm, self.active_tempo, "indefinite",
                                                   self.active_synch_interval)
            instrument = code.split("-")[0]
            track = code.split("-")[1]
            managed = False
            for i in range(len(self.active_instrument)):
                if track == self.active_track[i] and instrument == self.active_instrument[i]:
                    # remove the instrument
                    self.speaker_manager.stop_audio_track(code + "-" + str(self.active_tempo),
                                                          self.active_synch_interval)
                    self.active_track.pop(i)  # remove
                    self.active_instrument.pop(i)  # remove
                    managed = True
                    break
                elif instrument == self.active_instrument[i] and track != self.active_track[i]:
                    # change track
                    self.speaker_manager.switch_audio_track([code + "-" + str(self.active_tempo)],
                                                            [self.active_instrument[i] + "-" + self.active_track[
                                                                i] + "-" + str(self.active_tempo)],
                                                            -1,
                                                            self.active_synch_interval)
                    self.active_track[i] = track  # change track
                    managed = True

            if not managed:  # no instrument was stopped no instrument was changed track => start instrument
                self.speaker_manager.start_audio_track(code + "-" + str(self.active_tempo), -1,
                                                       self.active_synch_interval)
                self.active_track.append(track)
                self.active_instrument.append(instrument)
                if self.__music_player_thread is not None and self.__music_player_thread.is_alive():
                    self.__music_player_thread.kill()
                    self.stick_manager.stop_animation()
                # Here we kill the previous process if existing
                self.state = MusicPlayingState()

            if self.active_instrument == []:  # if no active elements at this point stop animation
                self.stick_manager.stop_animation()
                self.state = IdleState()
                # Here we thank the user
                self.speaker_manager.start_track_and_wait("aftermusic")
                print("stopping all animations")

        elif code.startswith("t:"):
            code = code[2:]  # remove first 2 char
            # change tempo
            self.speaker_manager.switch_audio_track(
                [i + "-" + t + "-" + str(code) for i in self.active_instrument for t in self.active_track],
                [i + "-" + t + "-" + str(self.active_tempo) for i in self.active_instrument for t in self.active_track],
                -1,
                self.active_synch_interval)
            self.stick_manager.change_tempo(code)
            self.active_tempo = int(code)
            active_synch_interval = (60000 / self.active_tempo) * 4


if __name__ == "__main__":
    mc: MusicController = MusicController()
    inpt = ""
    while inpt != "q":
        inpt = input("Type p for PersonDetected, i:instrument-track for Instrument, t:tempo for tempo: ")
        if inpt == "p":
            mc.handle_person(True)
        else:
            mc.handle_code(inpt)
    print("exit")
