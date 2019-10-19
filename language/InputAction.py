class InputAction:

	def __init__(self, targetContext, targetID):
		self.targetContext = targetContext
		self.targetID = targetID

	def getString(self):
		if self.pattern is not None:
			string = self.pattern.getString()
			return string
		else: return None

	def getPattern(self):
		return self.pattern

	def setPattern(self, pattern):
		self.pattern = pattern 

	def getTargetContext(self):
		return self.targetContext

	def setTargetContext(self, targetContext):
		self.targetContext = targetContext

	def getTargetID(self):
		return self.targetID

	def setTargetID(self, targetID):
		self.targetID = targetID
