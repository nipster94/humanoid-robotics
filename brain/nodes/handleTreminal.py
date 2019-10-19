#!/usr/bin/env python
import rospy
import rospkg
import yaml
from pyconfigstore import ConfigStore
from brain.srv import RequestTreminal,RequestTreminalRequest,RequestTreminalResponse

class HandleTreminal():
    def __init__(self):
        rospy.init_node('TreminalApplication')
        rospack = rospkg.RosPack()
        self.initialPoint = True
        path = rospack.get_path('brain') + '/resources/users.yaml'

        self.terminalService = rospy.Service('/hubert_brain/treminal',RequestTreminal,self.handle_terminal_service)

        self.data = self.read_yaml(path)

        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()


    def read_yaml(self,path):
        with open(path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        print data_loaded
        print data_loaded["users"][0]
        print data_loaded["users"][0]["user_name"]
        print data_loaded["users"][0]["credentials"]

        return data_loaded

    def handle_terminal_service(self,request):
        responce = RequestTreminalResponse()
        if(request.open_terminal):
            self.open_treminal()
            # haveAccess,name = self.check_access()


            responce.access = True
            responce.name = "temp"

        return responce

    def open_treminal(self):
        conf = ConfigStore("EmailCLI")

    def check_access(self):
        return True,"temp"


if __name__ == '__main__':
    HandleTreminal()




