#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from brain.msg import Access,Feedback

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

        self.generateHelloDialog()             #0
        self.generateInterrogation()           #1
        self.generateWelcomeDialog()           #2
        self.generateNotWelcomeDialog()        #3
        self.generateWarningDialog()           #4
        self.generateSecondWarningDialog()     #5
        self.generateFinalWarningDialog()      #6
        self.generateInterrogationWarnings()   #7

        self.agent.setDialogList(self.dialogList)

        self.speech = SpeechSynthesizer()

        rospy.init_node("LanguageAgent")
        rospy.Subscriber('/hubert_brain/say_hello',Bool, self.say_hello_callback)
        rospy.Subscriber('/hubert_brain/start_interrogation',Bool,self.start_interrogation)
        rospy.Subscriber('/hubert_brain/access_details', Access, self.take_decision)
        rospy.Subscriber('/hubert_brain/feedback',Feedback,self.handle_feedback)

        self.agent_feedback = rospy.Publisher('/hubert_brain/feedback',Feedback,queue_size=1)

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
            rospy.loginfo("SAY HELLO")
            self.got_data = True
            startContext = self.agent.getDialogList()[0].getContext()
            startID = self.agent.getDialogList()[0].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
            self.got_data = False

    def start_interrogation(self,data):
        if(data.data):
            rospy.loginfo("START INTERROGATION")
            self.got_data = True
            startContext = self.agent.getDialogList()[1].getContext()
            startID = self.agent.getDialogList()[1].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
            self.got_data = False

    def take_decision(self,data):
        feedback = Feedback()
        self.got_data = True
        if(data.access_granted):
            rospy.loginfo("WELCOME")
            startContext = self.agent.getDialogList()[2].getContext()
            startID = self.agent.getDialogList()[2].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
            self.speech.speak(data.user_name)
            feedback.agent_feedback = "initial welcome"
        else:
            rospy.logerr("NOT WELCOME")
            startContext = self.agent.getDialogList()[3].getContext()
            startID = self.agent.getDialogList()[3].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
            feedback.agent_feedback = "initial warning"

        self.agent_feedback.publish(feedback)
        self.got_data = False

    def handle_feedback(self,data):
        feedback = data

        if(feedback.treminal_feedback is not "" and
                feedback.treminal_feedback == "1 warning"):
            startContext = self.agent.getDialogList()[7].getContext()
            startID = self.agent.getDialogList()[7].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)
        elif(feedback.treminal_feedback is not "" and
                feedback.treminal_feedback == "2 warning"):
            startContext = self.agent.getDialogList()[7].getContext()
            startID = self.agent.getDialogList()[7].getDialogItemList()[1].getID()
            self.agent.start(startContext, startID)
        elif(feedback.brain_feedback is not  "" and
                feedback.brain_feedback == "1 warning"):
            startContext = self.agent.getDialogList()[4].getContext()
            startID = self.agent.getDialogList()[4].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)

            reply = Feedback()
            reply.agent_feedback = "1st warning issued"
            self.agent_feedback.publish(reply)
        elif(feedback.brain_feedback is not  "" and
                feedback.brain_feedback == "2 warning"):
            startContext = self.agent.getDialogList()[5].getContext()
            startID = self.agent.getDialogList()[5].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)

            reply = Feedback()
            reply.agent_feedback = "2nd warning issued"
            self.agent_feedback.publish(reply)
        elif(feedback.brain_feedback is not  "" and
                feedback.brain_feedback == "3 warning"):
            startContext = self.agent.getDialogList()[6].getContext()
            startID = self.agent.getDialogList()[6].getDialogItemList()[0].getID()
            self.agent.start(startContext, startID)

            reply = Feedback()
            reply.agent_feedback = "3rd warning issued"
            self.agent_feedback.publish(reply)
        else:
            rospy.logwarn("something else was passed")

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

        waitingTime = 1.0
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
        pattern = Pattern('Welcome')
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

    def generateInterrogationWarnings(self):
        dialogItemList = []
        finalWarningDialog = Dialog('InterrogationWarnings')

        itemIW1 = OutputItem('IW1')
        outputAction = OutputAction('', '')
        pattern = Pattern('This is your first warning. You have two attemps left')
        outputAction.setPattern(pattern)
        itemIW1.setOutputAction(outputAction)
        dialogItemList.append(itemIW1)

        itemIW2 = OutputItem('IW2')
        outputAction = OutputAction('', '')
        pattern = Pattern('This is your second warning. You better get this right')
        outputAction.setPattern(pattern)
        itemIW2.setOutputAction(outputAction)
        dialogItemList.append(itemIW2)

        finalWarningDialog.setDialogItemList(dialogItemList)
        self.dialogList.append(finalWarningDialog)

if __name__ == '__main__':
    AgentMain()
