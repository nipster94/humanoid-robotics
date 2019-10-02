#!/usr/bin/env python
import rospy
import rospkg
import math
from std_msgs.msg import UInt16,UInt16MultiArray
from geometry_msgs.msg import PointStamped

class ConvertCoordinates():
    def __init__(self):
        rospy.init_node('ConvertCoordinates')

        rospy.Subscriber('/img_points',PointStamped, self.target_callback)
        self.neck_angle_pub = rospy.Publisher("/servo_neck", UInt16MultiArray, queue_size=1)
        self.body_angle_pub = rospy.Publisher("/servo_body", UInt16, queue_size=1)
        self.neck_angles = []
        self.gotNewData = False

        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_neck_angle()
            self.rate.sleep()
            # rospy.spin()


    def target_callback(self,data):

        print 'in tracking'
        self.gotNewData = True
        x = data.point.x
        y = data.point.y
        w = 1
        h = 1
        angle_x, angle_y, distance = self.trackface(x,y,w,h)

        self.neck_angles = [angle_x,angle_y]

    def publish_neck_angle(self):
        if self.gotNewData:
            print self.neck_angles
            neck_msg = UInt16MultiArray()
            for i in self.neck_angles:
                neck_msg.data.append(i)

            self.neck_angle_pub.publish(neck_msg)
            self.gotNewData = False



        # if not self.neck_angle:
        #     neck_msg = UInt16MultiArray()
        #     neck_msg.data.append(1)
        #
        #     self.neck_angle_pub.publish(neck_msg)

        # self.neck_angle = []

    def trackface(self,x, y, w, h):
        print 'inside track face'

        height = 480
        width = 640
        f = 3.2                 # focal length
        e = 25.4 / 300          # pixel size

        angle_x = math.atan((x - 0.5 * width) / (f / e)) * 180 / math.pi + 90
        angle_y = math.atan((y - 0.5 * height) / (f / e)) * 180 / math.pi + 90

        facesize = 170
        distance = facesize * f / (w * e)

        return angle_x, angle_y, distance


if __name__ == '__main__':
    ConvertCoordinates()

    
    