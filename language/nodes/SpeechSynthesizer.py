import pyttsx3

class SpeechSynthesizer:

	engine = pyttsx3.init()
	rate = engine.getProperty('rate')
	engine.setProperty('rate', rate - 50)

	def speak(self, string):
		self.engine.say(string)	
		self.engine.runAndWait()
		print('\n' + string)
