class Dialog:

	def __init__(self, context):
		self.context = context

	def initialize(self, ownerAgent):
		for dialogItem in self.dialogItemList:
			dialogItem.initialize(ownerAgent)

	def getContext(self):
		return self.context

	def setContext(self, context):
		self.context = context

	def getDialogItemList(self):
		return self.dialogItemList

	def setDialogItemList(self, dialogItemList):
		self.dialogItemList = dialogItemList
