#include "face_detection.h"

int main(int argc, char** argv)
{
    ROS_INFO("Starting face detection...");
    ros::init(argc, argv, "ros_face_detection");
    ROSFaceDetection node;
    node.execute();
    return 0;
}
