#!/usr/bin/env python
import rospy
import rospkg
import math
from std_msgs.msg import UInt16,UInt16MultiArray
from geometry_msgs.msg import PointStamped
from face_detection.msg import MoveBase

class ConvertCoordinates():
    def __init__(self):
        rospy.init_node('ConvertCoordinates')

        rospy.Subscriber('/face_detection/img_points',PointStamped, self.target_callback)
        rospy.Subscriber('/face_detection/move_base',MoveBase,self.move_base_callback)
        self.neck_angle_pub = rospy.Publisher("/servo_neck", UInt16MultiArray, queue_size=1)
        self.body_angle_pub = rospy.Publisher("/servo_body", UInt16, queue_size=1)


        self.neck_angles = []
        self.current_angle_x = 0
        self.current_angle_y = 0
        self.pre_angle_x = 0
        self.pre_angle_y = 0
        self.body_angle = 90             # Intial angle of the body servo
        self.init = True
        self.gotNewData = False
        self.move_base = False

        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_neck_angle()
            self.publish_body_angle()
            self.rate.sleep()

    def target_callback(self,data):
        x = data.point.x
        y = data.point.y
        w = 1
        h = 1
        self.current_angle_x, self.current_angle_y, distance = self.trackface(x,y,w,h)

        if self.init:
            self.pre_angle_x = self.current_angle_x
            self.pre_angle_y = self.current_angle_y
            self.gotNewData = True
            self.init = False

        if abs(self.pre_angle_x - self.current_angle_x) > 30 and \
                abs(self.pre_angle_y - self.current_angle_y) > 30:

            print '======================'
            print self.pre_angle_x , self.current_angle_x
            print abs(self.pre_angle_x - self.current_angle_x)
            print self.pre_angle_y , self.current_angle_y
            print abs(self.pre_angle_y - self.current_angle_y)

            self.gotNewData = True
            self.neck_angles = [self.current_angle_x,
                                self.current_angle_y]

            # self.neck_angles = [angle_x,angle_y]
    def move_base_callback(self,data):
        if data.move_base:
            if data.turn_left:
                self.body_angle += 5
            else:
                self.body_angle -= 5

    def trackface(self,x, y, w, h):
        height = 480
        width = 640
        f = 3.2                 # focal length
        e = 25.4 / 300          # pixel size

        angle_x = math.atan((x - 0.5 * width) / (f / e)) * 180 / math.pi + 90
        angle_y = math.atan((0.5 * height - y) / (f / e)) * 180 / math.pi + 90

        facesize = 170
        distance = facesize * f / (w * e)

        return angle_x, angle_y, distance

    def publish_neck_angle(self):
        if self.gotNewData:
            # print self.neck_angles
            neck_msg = UInt16MultiArray()
            for i in self.neck_angles:
                neck_msg.data.append(i)

            self.neck_angle_pub.publish(neck_msg)
            self.gotNewData = False

    def publish_body_angle(self):
        if self.move_base:
            body_msg = UInt16()
            body_msg.data = self.body_angle

            self.body_angle_pub.publish(body_msg)


if __name__ == '__main__':
    ConvertCoordinates()

    
    