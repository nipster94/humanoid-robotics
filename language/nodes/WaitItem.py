from DialogItem import *
import time

class WaitItem(DialogItem):

	def __init__(self, _id, waitingTime):
		self._id = _id
		self.waitingTime = waitingTime

	def run(self):
		time.sleep(self.waitingTime)
		targetContext = self.outputAction.getTargetContext()
		targetID = self.outputAction.getTargetID()
		return targetContext, targetID, True

	def getOutputAction(self):
		return self.outputAction

	def setOutputAction(self, outputAction):
		self.outputAction = outputAction

	def getWaitingTime(self):
		return self.waitingTime

	def setWaitingTime(self, waitingTime):
		self.waitingTime = waitingTime
