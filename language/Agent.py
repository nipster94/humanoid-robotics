from SpeechSynthesizer import *
from WorkingMemory import *

class Agent:
	
	speech = SpeechSynthesizer()

	def initialize(self):
		for dialog in self.dialogList:
			dialog.initialize(self)

	def sendSpeechOutput(self, string):
		self.speech.speak(string)

	def getDialogList(self):
		return self.dialogList 

	def setDialogList(self, dialogList):
		self.dialogList = dialogList

	def findCurrentDialog(self):
		currentDialog = [dialog for dialog in self.dialogList if dialog.getContext() == self.workingMemory.getCurrentContext()]
		if len(currentDialog) > 0: 
			return currentDialog[0]
		else: return None

	def findCurrentDialogItem(self):
		currentDialog = self.findCurrentDialog()
		if currentDialog is not None:
			currentDialogItem = [dialogItem for dialogItem in currentDialog.getDialogItemList() if dialogItem.getID() == self.workingMemory.getCurrentID()] 
			if len(currentDialogItem) > 0: 
				return currentDialogItem[0]
			else: return None

		else: return None	

	def currentIDChanged(self):
		currentDialogItem = self.findCurrentDialogItem()
		if currentDialogItem is not None:
			targetContext, targetID, boolean = currentDialogItem.run()
			if targetContext != self.workingMemory.getCurrentContext():
				self.workingMemory.setCurrentContext(targetContext)
			self.workingMemory.setCurrentID(targetID) # seems to work

	def start(self, startContext, startID):
		self.initialize()
		self.workingMemory = WorkingMemory()
		self.workingMemory.setOwnerAgent(self)	
		self.workingMemory.setCurrentContext(startContext)
		self.workingMemory.setCurrentID(startID)

