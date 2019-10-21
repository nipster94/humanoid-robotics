#!/usr/bin/env python
import rospy
import rospkg
from brain.srv import RequestTreminal,RequestTreminalResponse
from brain.msg import Feedback

import yaml
import six
import os
import getpass
from pyfiglet import figlet_format

try:
    from termcolor import colored
except ImportError:
    colored = None

class HandleTreminal():
    def __init__(self):
        rospy.init_node('TreminalApplication')
        rospack = rospkg.RosPack()
        self.initialPoint = True
        path = rospack.get_path('brain') + '/resources/users.yaml'

        self.terminalService = rospy.Service('/hubert_brain/terminal',RequestTreminal,self.handle_terminal_service)

        self.feedback_pub = rospy.Publisher('/hubert_brain/feedback',Feedback,queue_size=1)

        self.data = self.read_yaml(path)
        self.interrogationDone = False

        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if(self.interrogationDone):
                rospy.sleep(4)
                os.system('clear')
                self.interrogationDone = False
            self.rate.sleep()


    def read_yaml(self,path):
        with open(path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        return data_loaded

    def handle_terminal_service(self,request):
        os.system('clear')
        responce = RequestTreminalResponse()
        if(request.open_terminal):
            haveAccess, name = self.open_treminal()
            # haveAccess,name = self.check_access()
            responce.access = haveAccess
            responce.name = name

        return responce

    def open_treminal(self):
        self.load_welcome()
        self.load_instructions()

        while raw_input():
            pass

        return self.load_questions()

    def load_welcome(self):
        log_msg_2 = '''
                            Welcome to the new Authentication system of the Department of TIF at 
                            Chalmers University of Technology
                            '''
        self.log("Hubert the Guard Robot", color="blue", figlet=True)
        self.log(log_msg_2, "green")

    def load_instructions(self):
        log_msg_1 = '''\t\t  Instructions:
                     \t\t 1. Enter your USER NAME -- You will be given 3 chances               \n   
                     \t\t 2. If your user name is registered, then you may enter your PASSWORD 
                     \t\t    Again you will be given 3 chances
                  Upon successful authentication you will be grante access to this facility.   \n
                  If you fail to comply you will be given 3 warning and will take deadly force  
                  to deadly force to remove you from the facility.  
                  
                  You will be given 20 seconds to submit your credentials. \n
                  Please press ENTER when you are read...           
                  '''
        self.log(log_msg_1, "yellow")

    def load_questions(self):
        question_1 = "Enter your USER NAME: \n"
        self.log(question_1, color="white")

        user_name = raw_input()
        no_of_attempts = 1
        access_granted = False
        got_usr_name = False
        user_index = 0

        while (not access_granted and not no_of_attempts >=3):
            print no_of_attempts
            if(got_usr_name):
                pswd = getpass.getpass('Please enter your PASSWORD:')
                if (pswd == self.data["users"][user_index]["credentials"]["password"]):
                    access_granted = True
                    no_of_attempts = 3
                else:
                    feedback = Feedback()
                    if(no_of_attempts == 1):
                        feedback.treminal_feedback = "1 warning"
                    elif(no_of_attempts == 2):
                        feedback.treminal_feedback = "2 warning"

                    self.feedback_pub.publish(feedback)

                    no_of_attempts += 1

                    access_granted = False
                    query = "YOU HAVE ANOTHER " + str(3 - no_of_attempts) + " ATTEMPS TO ENTER YOUR PASSWORD"
                    self.log(query, color="red")
            else:
                for index,item in enumerate(self.data["users"]):
                    if(user_name == item["user_name"]):
                        print user_name
                        no_of_attempts = 0
                        user_index = index
                        got_usr_name =True
                        pswd = getpass.getpass('Please enter your PASSWORD:')

                        if(pswd == item["credentials"]["password"]):
                            access_granted = True
                            no_of_attempts = 4
                        else:
                            no_of_attempts += 1

                            feedback = Feedback()
                            if (no_of_attempts == 1):
                                feedback.treminal_feedback = "1 warning"
                            elif (no_of_attempts == 2):
                                feedback.treminal_feedback = "2 warning"

                            self.feedback_pub.publish(feedback)

                            access_granted = False
                            query = "YOU HAVE ANOTHER " + str(3 - no_of_attempts) + "ATTEMPS"
                            self.log(query, color="red")

                if(not got_usr_name):
                    feedback = Feedback()
                    if (no_of_attempts == 1):
                        feedback.treminal_feedback = "1 warning"
                    elif (no_of_attempts == 2):
                        feedback.treminal_feedback = "2 warning"

                    no_of_attempts += 1

                    self.feedback_pub.publish(feedback)

                    query = "YOU HAVE ANOTHER " + str(3 - no_of_attempts) + " ATTEMPS TO ENTER YOUR USER NAME"
                    self.log(query, color="red")
                    user_name = raw_input()


        self.interrogationDone = True

        if(no_of_attempts >= 3 and not access_granted):
            self.give_warning()
            return False,""
        elif(access_granted):
            return True, self.data["users"][user_index]["credentials"]["name"]


    def give_warning(self):
        query = "YOU HAVE FAILED TO PROVIDE PROPER CREDENTIALS"
        self.log(query, color="red")

    def log(self, string, color, font="slant", figlet=False):
        if colored:
            if not figlet:
                six.print_(colored(string, color))
            else:
                six.print_(colored(figlet_format(
                    string, font=font), color))
        else:
            six.print_(string)

    def check_access(self):
        return True,"temp"


if __name__ == '__main__':
    HandleTreminal()




