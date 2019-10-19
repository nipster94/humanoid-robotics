#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

from Agent import *
from Dialog import *
from DialogItem import *
from OutputItem import *
from WaitItem import *
from OutputAction import *
from Pattern import *

class AgentMain():
    def __init__(self):
        self.agent = Agent()
        self.dialogList = []

        self.generateHelloDialog()
        self.generateInterrogation()
        self.generateWelcomeDialog()
        self.generateNotWelcomeDialog()
        self.generateWarningDialog()
        self.generateSecondWarningDialog()
        self.generateFinalWarningDialog()

        self.agent.setDialogList(self.dialogList)

        rospy.init_node("LanguageAgent")
        rospy.Subscriber('/hubert_brain/say_hello',Bool, self.say_hello_callback)
        rospy.Subscriber('/hubert_brain/start_interrogation',Bool,self.start_interrogation)

        self.got_data = False

        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            while self.got_data:
                pass

            if not self.got_data:
                rospy.loginfo("Waiting for interrogation")
            self.rate.sleep()

    def say_hello_callback(self,data):
        if (data.data):
            self.got_data = True
            startContext = self.agent.getDialogList()[0].getContext()
            startID = self.agent.getDialogList()[0].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
            self.got_data = False

    def start_interrogation(self,data):
        if(data.data):
            rospy.loginfo("START INTERROGATION")
            self.got_data = True
            startContext = self.agent.getDialogList()[2].getContext()
            startID = self.agent.getDialogList()[2].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
            self.got_data = False


    def generateHelloDialog(self):
        dialogItemList = []
        helloDialog = Dialog('HelloDialog')

        itemHD1 = OutputItem('HD1')
        outputAction = OutputAction('', '')
        pattern = Pattern('Hello.')
        outputAction.setPattern(pattern)
        itemHD1.setOutputAction(outputAction)
        dialogItemList.append(itemHD1)

        helloDialog.setDialogItemList(dialogItemList)
        self.dialogList.append(helloDialog)

    def generateInterrogation(self):
        dialogItemList = []
        interrogationDialog = Dialog('Interrogation')

        itemI1 = OutputItem('I1')
        outputAction  = OutputAction('Interrogation','I2')
        pattern = Pattern('You have enterted restricted area')
        outputAction.setPattern(pattern)
        itemI1.setOutputAction(outputAction)
        dialogItemList.append(itemI1)

        waitingTime = 5.0
        itemI2 = WaitItem('I2', waitingTime)
        outputAction = OutputAction('Interrogation', 'I3')
        itemI2.setOutputAction(outputAction)
        dialogItemList.append(itemI2)

        itemI3 = OutputItem('I3')
        outputAction =  OutputAction('', '')
        pattern = Pattern('Please enter your user name and password in the terminal')
        outputAction.setPattern(pattern)
        itemI3.setOutputAction(outputAction)
        dialogItemList.append(itemI3)

        interrogationDialog.setDialogItemList(dialogItemList)
        self.dialogList.append(interrogationDialog)

    def generateLoginDialog(self):

        dialogItemList = []
        loginDialog = Dialog('LoginDialog')

        itemLD1 = OutputItem('LD1')
        outputAction = OutputAction('', '')
        pattern = Pattern('Please enter your ID number.')
        outputAction.setPattern(pattern)
        itemLD1.setOutputAction(outputAction)
        dialogItemList.append(itemLD1)

        loginDialog.setDialogItemList(dialogItemList)
        self.dialogList.append(loginDialog)

    def generateWelcomeDialog(self):

        dialogItemList = []
        welcomeDialog = Dialog('WelcomeDialog')

        itemWD1 = OutputItem('WD1')
        outputAction = OutputAction('', '')
        pattern = Pattern('Welcome.')
        outputAction.setPattern(pattern)
        itemWD1.setOutputAction(outputAction)
        dialogItemList.append(itemWD1)

        welcomeDialog.setDialogItemList(dialogItemList)
        self.dialogList.append(welcomeDialog)

    def generateNotWelcomeDialog(self):

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
        self.dialogList.append(notWelcomeDialog)

    def generateWarningDialog(self):

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
        self.dialogList.append(warningDialog)

    def generateSecondWarningDialog(self):

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
        self.dialogList.append(secondWarningDialog)

    def generateFinalWarningDialog(self):

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
        self.dialogList.append(finalWarningDialog)

if __name__ == '__main__':
    AgentMain()
