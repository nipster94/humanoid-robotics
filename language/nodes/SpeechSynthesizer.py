import pyttsx

class SpeechSynthesizer():
    def onStart(self,name):
        print name

    def speak(self,string):
        engine = pyttsx.init()
        engine.connect('started-utterance',self.onStart)

        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate - 50)

        engine.say(string,name=string)
        engine.runAndWait()
