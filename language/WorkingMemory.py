class WorkingMemory:

	def __init__(self):
		self.currentContext = ''
		self.currentID = ''

	def setOwnerAgent(self, ownerAgent):
		self.ownerAgent = ownerAgent

	def getCurrentContext(self):
		return self.currentContext

	def setCurrentContext(self, currentContext):
		self.currentContext = currentContext

	def getCurrentID(self):
		return self.currentID

	def setCurrentID(self, currentID):
		self.currentID = currentID
		self.ownerAgent.currentIDChanged()



	
