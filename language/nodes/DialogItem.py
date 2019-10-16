class DialogItem:

	def initialize(self, ownerAgent):
		self.ownerAgent = ownerAgent

	def run(self):
		return True

	def getID(self):
		return self._id

	def setID(self, _id):
		self._id = _id
