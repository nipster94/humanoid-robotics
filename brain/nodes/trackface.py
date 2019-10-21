#!/usr/bin/env python
import rospy
import rospkg
import math
from std_msgs.msg import UInt16,UInt16MultiArray,Bool
from geometry_msgs.msg import PointStamped
from face_detection.msg import MoveBase,Face

class FaceTracker():
    def __init__(self):
        rospy.init_node('FaceTracker')

        self.current_angle_x = 0
        self.current_angle_y = 0
        self.pre_angle_x = 0
        self.pre_angle_y = 0

        self.body_angle = 90             # Intial angle of the body servo
        self.pan_angle = 90              # Intial angle of the pan servo
        self.tilt_angle = 90             # Intial angle of the tilt servo

        self.init = True
        self.gotNewData = False
        self.move_base = False
        self.face_found = False

        rospy.Subscriber('/face_detection/img_location',Face, self.target_callback)
        rospy.Subscriber('/face_detection/move_base',MoveBase,self.move_base_callback)
        rospy.Subscriber('/face_detection/face_found',Bool, self.face_found_callback)

        self.pan_pub = rospy.Publisher("/servo_neck_rot", UInt16, queue_size=1)
        self.tilt_pub = rospy.Publisher("/servo_neck_tilt",UInt16,queue_size=1)
        self.body_angle_pub = rospy.Publisher("/servo_body", UInt16, queue_size=1)

        self.rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_neck_angle()
            self.publish_body_angle()
            self.rate.sleep()

    def face_found_callback(self,data):
        print data
        print self.face_found
        if(data.data):
            self.face_found = True

    def target_callback(self,data):
        if(self.face_found):
            x = data.point.point.x
            y = data.point.point.y
            w = data.height.data
            h = data.width.data
            self.current_angle_x, self.current_angle_y, distance = self.trackface(x,y,w,h)

            print self.current_angle_x, self.current_angle_y

            if self.init:
                self.pre_angle_x = self.current_angle_x
                self.pre_angle_y = self.current_angle_y
                self.gotNewData = True
                self.init = False

            if abs(self.pre_angle_x - self.current_angle_x) > 30 and \
                    abs(self.pre_angle_y - self.current_angle_y) > 30:

                # print self.pre_angle_x , self.current_angle_x
                # print abs(self.pre_angle_x - self.current_angle_x)
                # print self.pre_angle_y , self.current_angle_y
                # print abs(self.pre_angle_y - self.current_angle_y)

                self.gotNewData = True
                self.pan_angle = self.current_angle_x
                self.tilt_angle = self.current_angle_y

            # self.neck_angles = [angle_x,angle_y]
    def move_base_callback(self,data):
        if self.face_found:
            print data
        # if data.move_base.data:
        #     if data.turn_left:
        #         self.body_angle += 5
        #     else:
        #         self.body_angle -= 5

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
            pan_msg = UInt16()
            pan_msg.data = self.pan_angle

            tilt_msg = UInt16()
            tilt_msg.data = self.tilt_angle

            self.pan_pub.publish(pan_msg)
            self.tilt_pub.publish(tilt_msg)

            self.gotNewData = False

    def publish_body_angle(self):
        if self.move_base:
            body_msg = UInt16()
            body_msg.data = self.body_angle

            self.body_angle_pub.publish(body_msg)


if __name__ == '__main__':
    FaceTracker()

    
    