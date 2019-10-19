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
        self.generateWorldDialog()
        self.generateInterrogation()

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
        self.dialogList.append(helloDialog)

    def generateWorldDialog(self):

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
        self.dialogList.append(worldDialog)

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


if __name__ == '__main__':
    AgentMain()
