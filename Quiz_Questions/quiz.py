from pygame import mixer

class QuizController:

    __question_id: int
    __questions:list
    __answers: list

    def __init__(self):
        mixer.pre_init(44100, 16, 2, 4096) #frequency, size, channels, buffersize
        mixer.init()
        self.__question_id=0
        self.__questions = ['qn1.acc']
        self.__answers = ["october-6-1920"]

        

    def start_quiz(self):
        #play audio saying welcome to Quiz
        self.ask_quiz()

    def ask_quiz(self):
        if self.__question_id <= len(self.__questions):
            
           sound = mixer.Sound(self.__questions[self.__question_id])
           sound.play()
        else:
            print("play audio saying Thank you!")

    def conduct_quiz(self):
        if answer == self.__answers[self.__question_id]:
            print("correct")
            self.__question_id += 1
            self.ask_quiz()
        else:
            print("wrong")
            self.__question_id += 1
            self.ask_quiz()

        



if __name__ == '__main__':
    
    q = QuizController()
    q.start_quiz()
