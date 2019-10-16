
from Agent import *
from Dialog import *
from DialogItem import *
from OutputItem import *
from WaitItem import *
from OutputAction import *
from Pattern import *

agent = Agent()

dialogList = []

def generateHelloDialog():
	
	dialogItemList = []
	helloDialog = Dialog('HelloDialog')
			
	itemHD1 = OutputItem('HD1')
	outputAction = OutputAction('HelloDialog', 'HD2')
	pattern = Pattern('Hello')
	outputAction.setPattern(pattern)  
	itemHD1.setOutputAction(outputAction)
	dialogItemList.append(itemHD1)

	waitingTime = 5.0
	itemHD2 = WaitItem('HD2', waitingTime)
	outputAction = OutputAction('WorldDialog', 'WD2')
	itemHD2.setOutputAction(outputAction)
	dialogItemList.append(itemHD2)

	helloDialog.setDialogItemList(dialogItemList)
	dialogList.append(helloDialog)

def generateWorldDialog():
	
	dialogItemList = []
	worldDialog = Dialog('WorldDialog')

	itemWD1 = OutputItem('WD1')
	outputAction = OutputAction('WorldDialog', 'WD2')
	pattern = Pattern('World')
	outputAction.setPattern(pattern)  
	itemWD1.setOutputAction(outputAction)
	dialogItemList.append(itemWD1)	

	itemWD2 = OutputItem('WD2')
	outputAction = OutputAction('', '')
	pattern = Pattern('Again')
	outputAction.setPattern(pattern)  
	itemWD2.setOutputAction(outputAction)
	dialogItemList.append(itemWD2)

	worldDialog.setDialogItemList(dialogItemList)
	dialogList.append(worldDialog)

generateHelloDialog()
generateWorldDialog()
		
agent.setDialogList(dialogList)

startContext = agent.getDialogList()[0].getContext()
startID = agent.getDialogList()[0].getDialogItemList()[0].getID()
agent.start(startContext, startID)

