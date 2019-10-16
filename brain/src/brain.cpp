#include "brain.h"

HubertBrain::HubertBrain():
    nh_("~")
{
    face_found = false;
    previous_face_found = false;

//    temp_nh_.setCallbackQueue(queue);

//    nh_.setCallbackQueue(&queue);




    neck_loop_= nh_.advertise<std_msgs::UInt16MultiArray>("/servo_neck",1);
    body_loop_ = nh_.advertise<std_msgs::UInt16>("/servo_body",1);

    face_found_sub = nh_.subscribe("/face_detection/face_found", 1, &HubertBrain::faceFoundCallback,this);

//    global_spinner->start();

//    global_spinner(0,&queue);



//    global_spinner.start();

//    ros::AsyncSpinner tempSpinner(0,&queue);



//    ROS_INFO("Spinner enabled");

    pan_tilt_lb = 60;
    pan_tilt_ub = 150;

    body_lb = 60;
    body_ub = 120;


}

void HubertBrain::execute(){
    ROS_INFO("STARTING HUBERT BRAIN!!!");
    ros::Rate rate(10);

    while(ros::ok()){
        checkRobotStates();
        rate.sleep();
        ros::spinOnce();
    }

//    spinner.stop();
//    global_spinner->stop();
//    ROS_INFO("Spinner disabled");
}

void HubertBrain::faceFoundCallback(const std_msgs::Bool &msg){
    ROS_INFO("GOT FACE FOUND");
    face_found = msg.data;
    if(face_found != previous_face_found){
        if(robotState == RobotState::Idling){
            robotState = RobotState::Tracking;
            state_changed = true;
        }

        previous_face_found = face_found;
    }
}

void HubertBrain::idlingLoop(){
    
    uint tilt = 90;
    uint pan = 90;
    uint body = 90;

    bool reachTop = false;
    bool reachMiddle = true;
    bool reachBottom = false;
    bool goingUp = true;
    bool goingDown = false;
    
    ros::AsyncSpinner spinner(1);
    ros::Rate spinnerRate(0.5);
    spinner.start();
    while(!state_changed){
        std_msgs::UInt16MultiArray neck_msg;
        std_msgs::UInt16 body_msg;

        if(reachMiddle && tilt == 120 && pan == 120 && body == 140){
            reachMiddle = false;
            reachTop = true;
            goingDown = true;
            goingUp = false;
            tilt -= 3;
            pan -= 3;
            body -= 5;
        }
        else if (reachTop && tilt == 90 && pan == 90 && body == 90) {
            reachMiddle = true;
            reachTop = false;
            tilt -= 3;
            pan -= 3;
            body -= 5;
        }
        else if(reachMiddle && tilt == 60 && pan == 60 && body == 40){
            reachBottom = true;
            reachMiddle = false;
            goingDown = false;
            goingUp = true;
            tilt += 3;
            pan += 3;
            body += 5;
        }
        else if (reachBottom && tilt == 90 && pan == 90 && body == 90) {
            reachMiddle = true;
            reachBottom = false;
            tilt -= 3;
            pan -= 3;
            body -= 5;
        }
        else if(goingUp) {
            tilt += 3;
            pan += 3;
            body += 5;
        }
        else if (goingDown) {
            tilt -= 3;
            pan -= 3;
            body -= 5;
        }

        
//        uint neck_data[2] = {tilt,pan};
        neck_msg.data.push_back(tilt);
        neck_msg.data.push_back(pan);

//        neck_msg.data = neck_data;

//        uint body_data = body;
        body_msg.data = body;

        neck_loop_.publish(neck_msg);
        body_loop_.publish(body_msg);

        ROS_INFO("PUBLISHING.....");
        spinnerRate.sleep();
        ROS_INFO("DONE WAITING.....");
    }
    ROS_INFO("STOPING IDLING LOOP.....");
//    robotState = RobotState::Tracking;
    spinner.stop();
}


void HubertBrain::checkRobotStates(){
    if(robotState == RobotState::Idling){
        idlingLoop();
    }
    else if(robotState == RobotState::Tracking){

    }
    else if(robotState == RobotState::Interrogation){

    }
    else{

    }

}



