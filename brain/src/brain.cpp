#include "brain.h"

HubertBrain::HubertBrain():
    nh_("~")
{
    face_found = false;
    previous_face_found = false;

    face_found_sub = nh_.subscribe("face_detection/face_found", 1, &HubertBrain::faceFoundCallback,this);
}

void HubertBrain::execute(){
    ROS_INFO("STARTING HUBERT BRAIN!!!");
    ros::Rate rate(10);

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }
}

void HubertBrain::faceFoundCallback(const std_msgs::Bool &msg){
    face_found = msg.data;

    if(face_found != previous_face_found){
        if(robotState == RobotState::Idling)
            robotState == RobotState::Tracking;
    }
}

void HubertBrain::checkRobotStates(){
    if(robotState == RobotState::Idling){

    }
}



