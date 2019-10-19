from DialogItem import *

class InputItem(DialogItem):

	def __init__(self, _id, maximumRepetitionCount, finalFailureTargetContext, finalFailureTargetID):
		self._id = _id
		self.maximumRepetitionCount = maximumRepetitionCount
		self.finalFailureTargetContext
		self.finalFailureTargetID = finalFailureTargetID

	def run(self): # repetition count
		if repetitionCount > maximumRepetitionCount:
			return self.finalFailureTargetContext, self.finalFailureTargetID, True
		# 
		targetContext = self.inputAction.getTargetContext()
		targetID = self.inputAction.getTargetID()
		return targetContext, targetID, True

	def getInputAction(self):
		return self.inputAction

	def setInputAction(self, inputAction):
		self.inputAction = inputAction

	def getMaximumRepetitionCount(self):
		return self.maximumRepetitionCount

	def setMaximumRepetitionCount(self, maximumRepetitionCount):
		self.maximumRepetitionCount = maximumRepetitionCount

	def getFinalFailureTargetContext(self):
		return self.finalFailureTargetContext

	def setFinalFailureTargetContext(self, finalFailureTargetContext):
		self.finalFailureTargetContext = finalFailureTargetContext

	def getFinalFailureTargetID(self):
		return self.finalFailureTargetID

	def setFinalFailureTargetID(self, finalFailureTargetID):
		self.finalFailureTargetID = finalFailureTargetID
