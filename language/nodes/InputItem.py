from DialogItem import *

class InputItem(DialogItem):

	def __init__(self, _id):
		self._id = _id

	def run(self):

		#string = self.inputAction.getString()
		
		
	 
		inputString = # handle input
		isMatching, matchingPattern = self.inputAction.checkMatch(inputString)
		if isMathching:

			targetContext = self.inputAction.getTargetContext()
			targetID = self.inputAction.getTargetID()

			queryTerms = matchingPattern.getQueryTerms() # 
			
                        if len(queryTerms) > 0:
 
				self.addQueryTermsToWorkingMemory(queryTerms)
			
			return targetContext, targetID, True
			
		else: 



	def getInputAction(self):
		return self.inputAction

	def setInputAction(self, inputAction):
		self.inputAction = inputAction

