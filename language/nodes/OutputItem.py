from DialogItem import *

class OutputItem(DialogItem):

	def __init__(self, _id):
		self._id = _id

	def run(self):
		outputString = self.outputAction.getString()
		self.ownerAgent.sendSpeechOutput(outputString) 
		targetContext = self.outputAction.getTargetContext()
		targetID = self.outputAction.getTargetID()
		return targetContext, targetID, True

	def getOutputAction(self):
		return self.outputAction

	def setOutputAction(self, outputAction):
		self.outputAction = outputAction
		
