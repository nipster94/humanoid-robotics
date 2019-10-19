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
	outputAction = OutputAction('', '')
	pattern = Pattern('Hello.')
	outputAction.setPattern(pattern)  
	itemHD1.setOutputAction(outputAction)
	dialogItemList.append(itemHD1)

def generateLoginDialog():
	
	dialogItemList = []
	loginDialog = Dialog('LoginDialog')

	itemLD1 = OutputItem('LD1')
	outputAction = OutputAction('', '')
	pattern = Pattern('Please enter your ID number.')
	outputAction.setPattern(pattern)  
	itemLD1.setOutputAction(outputAction)
	dialogItemList.append(itemLD1)	

	loginDialog.setDialogItemList(dialogItemList)
	dialogList.append(loginDialog)

def generateWelcomeDialog():
	
	dialogItemList = []
	welcomeDialog = Dialog('WelcomeDialog')

	itemWD1 = OutputItem('WD1')
	outputAction = OutputAction('', '')
	pattern = Pattern('Welcome.')
	outputAction.setPattern(pattern)  
	itemWD1.setOutputAction(outputAction)
	dialogItemList.append(itemWD1)	

	welcomeDialog.setDialogItemList(dialogItemList)
	dialogList.append(welcomeDialog)

def generateNotWelcomeDialog():
	
	dialogItemList = []
	notWelcomeDialog = Dialog('NotWelcomeDialog')

	itemNW1 = OutputItem('NW1')
	outputAction = OutputAction('NotWelcomeDialog', 'NW2')
	pattern = Pattern('You do not have access here.')
	outputAction.setPattern(pattern)  
	itemNW1.setOutputAction(outputAction)
	dialogItemList.append(itemNW1)	
	
	waitingTime = 2.0
	itemNW2 = WaitItem('NW2', waitingTime)
	outputAction = OutputAction('NotWelcomeDialog', 'NW3')
	itemNW2.setOutputAction(outputAction)
	dialogItemList.append(itemNW2)

	itemNW3 = OutputItem('NW3')
	outputAction = OutputAction('', '')
	pattern = Pattern('All unauthorized personnel are forbidden to enter.')
	outputAction.setPattern(pattern)  
	itemNW3.setOutputAction(outputAction)
	dialogItemList.append(itemNW3)	

	notWelcomeDialog.setDialogItemList(dialogItemList)
	dialogList.append(notWelcomeDialog)

def generateWarningDialog():
	
	dialogItemList = []
	warningDialog = Dialog('WarningDialog')

	itemWD1 = OutputItem('WD1')
	outputAction = OutputAction('WarningDialog', 'WD2')
	pattern = Pattern('All unauthorized personnel must leave the area.')
	outputAction.setPattern(pattern)  
	itemWD1.setOutputAction(outputAction)
	dialogItemList.append(itemWD1)	
	
	waitingTime = 2.0
	itemWD2 = WaitItem('WD2', waitingTime)
	outputAction = OutputAction('WarningDialog', 'WD3')
	itemWD2.setOutputAction(outputAction)
	dialogItemList.append(itemWD2)

	itemWD3 = OutputItem('WD3')
	outputAction = OutputAction('', '')
	pattern = Pattern('Intruders will be eliminted.')
	outputAction.setPattern(pattern)  
	itemWD3.setOutputAction(outputAction)
	dialogItemList.append(itemWD3)	

	warningDialog.setDialogItemList(dialogItemList)
	dialogList.append(warningDialog)

def generateSecondWarningDialog():
	
	dialogItemList = []
	secondWarningDialog = Dialog('SecondWarningDialog')

	itemWD1 = OutputItem('WD1')
	outputAction = OutputAction('SecondWarningDialog', 'WD2')
	pattern = Pattern('I am armed.')
	outputAction.setPattern(pattern)  
	itemWD1.setOutputAction(outputAction)
	dialogItemList.append(itemWD1)	
	
	waitingTime = 2.0
	itemWD2 = WaitItem('WD2', waitingTime)
	outputAction = OutputAction('SecondWarningDialog', 'WD3')
	itemWD2.setOutputAction(outputAction)
	dialogItemList.append(itemWD2)

	itemWD3 = OutputItem('WD3')
	outputAction = OutputAction('', '')
	pattern = Pattern('You have been warned.')
	outputAction.setPattern(pattern)  
	itemWD3.setOutputAction(outputAction)
	dialogItemList.append(itemWD3)	

	secondWarningDialog.setDialogItemList(dialogItemList)
	dialogList.append(secondWarningDialog)
	
def generateFinalWarningDialog():
	
	dialogItemList = []
	finalWarningDialog = Dialog('FinalWarningDialog')

	itemFW1 = OutputItem('FW1')
	outputAction = OutputAction('FinalWarningDialog', 'FW2')
	pattern = Pattern('This is the final warning.')
	outputAction.setPattern(pattern)  
	itemFW1.setOutputAction(outputAction)
	dialogItemList.append(itemFW1)	
	
	waitingTime = 2.0
	itemFW2 = WaitItem('FW2', waitingTime)
	outputAction = OutputAction('FinalWarningDialog', 'FW3')
	itemFW2.setOutputAction(outputAction)
	dialogItemList.append(itemFW2)

	itemFW3 = OutputItem('FW3')
	outputAction = OutputAction('', '')
	pattern = Pattern('You have ten seconds to leave the area.')
	outputAction.setPattern(pattern)  
	itemFW3.setOutputAction(outputAction)
	dialogItemList.append(itemFW3)	

	finalWarningDialog.setDialogItemList(dialogItemList)
	dialogList.append(finalWarningDialog)


generateHelloDialog() 
generateWelcomeDialog()
generateNotWelcomeDialog()
generateWarningDialog()
generateSecondWarningDialog()
generateFinalWarningDialog()
		
agent.setDialogList(dialogList)

startContext = agent.getDialogList()[0].getContext()
startID = agent.getDialogList()[0].getDialogItemList()[0].getID()
agent.start(startContext, startID)

