#include "brain.h"

HubertBrain::HubertBrain():
    nh_("~")
{
    face_found = false;
    previous_face_found = false;

    neck_loop_= nh_.advertise<std_msgs::UInt16MultiArray>("/hubert_brain/servo_neck",1);
    body_loop_ = nh_.advertise<std_msgs::UInt16>("/hubert_brain/servo_body",1);

    // neck_loop_= nh_.advertise<std_msgs::UInt16MultiArray>("/servo_neck",1);
    // body_loop_ = nh_.advertise<std_msgs::UInt16>("/servo_body",1);
    
    say_hello_ = nh_.advertise<std_msgs::Bool>("/hubert_brain/say_hello",1);
    start_interrogation_ = nh_.advertise<std_msgs::Bool>("/hubert_brain/start_interrogation",1);
    
    face_found_sub = nh_.subscribe("/face_detection/face_found", 1, &HubertBrain::faceFoundCallback,this);

    pan_tilt_lb = 30;
    pan_tilt_ub = 150;
    pan_tilt_step_size = 15;

    body_lb = 60;
    body_ub = 120;
    
    ppl_passby_count = 0;

}

void HubertBrain::execute(){
    ROS_INFO("STARTING HUBERT BRAIN!!!");
    ros::Rate rate(10);
    panAngles_ = getPanAngles(pan_tilt_lb,pan_tilt_ub,pan_tilt_step_size);
    tiltAngles_ = getTiltAngles(pan_tilt_lb,pan_tilt_ub,pan_tilt_step_size);
    while(ros::ok()){
        checkRobotStates();
        rate.sleep();
        ros::spinOnce();
    }
}

void HubertBrain::faceFoundCallback(const std_msgs::Bool &msg){
    ROS_INFO("GOT FACE FOUND");
    face_found = msg.data;
    if(face_found != previous_face_found){ 
        if(robotState == RobotState::Idling){
            robotState = RobotState::Tracking;
            state_changed = true;
        }
        else if (robotState == RobotState::Tracking && face_found){
            ppl_passby_count += 1;
        }

        previous_face_found = face_found;
    }
    else{
        if(face_found && robotState == RobotState::Tracking){
//            continue;
        }
        
    }
}

std::vector<uint> HubertBrain::getPanAngles(int lb, int ub, int stepSize){
    bool goingUp = true;
    bool goingDown = false;

    std::vector<uint> panAngles;
    int currentAngle = 90;
    panAngles.push_back(currentAngle);

    int vecSize = ((ub-lb)/stepSize)*2;

    for (int i = 0; i < vecSize; i++){
        if(currentAngle < ub && goingUp){
            currentAngle += stepSize;
        }
        else if (currentAngle == ub && goingUp) {
            currentAngle -= stepSize;
            goingUp = false;
            goingDown = true;
        }
        else if (currentAngle > lb && goingDown) {
            currentAngle -= stepSize;
        }
        else if (currentAngle == lb && goingDown) {
            currentAngle += stepSize;
            goingUp = true;
            goingDown = false;
        }

        panAngles.push_back(currentAngle);
    }

    return panAngles;
}

std::vector<uint> HubertBrain::getTiltAngles(int lb, int ub, int stepSize){
    bool goingUp = true;
    bool goingDown = false;

    std::vector<uint> tiltAngles;
    int currentAngle = 90;
    tiltAngles.push_back(currentAngle);

    int vecSize = ((ub-lb)/stepSize)*2;

    for (int i = 0; i < vecSize; i++){
        if(currentAngle < ub && goingUp){
            currentAngle += stepSize;
        }
        else if (currentAngle == ub && goingUp) {
            currentAngle -= stepSize;
            goingUp = false;
            goingDown = true;
        }
        else if (currentAngle > lb && goingDown) {
            currentAngle -= stepSize;
        }
        else if (currentAngle == lb && goingDown) {
            currentAngle += stepSize;
            goingUp = true;
            goingDown = false;
        }

        tiltAngles.push_back(currentAngle);
    }

    return tiltAngles;
}


void HubertBrain::idlingLoop(){
    int count = 0;
    
    ros::AsyncSpinner spinner(1);
    ros::Rate spinnerRate(0.5);
    spinner.start();
    while(!state_changed){
        std_msgs::UInt16MultiArray neck_msg;
        std_msgs::UInt16 body_msg;

        count = count == panAngles_.size() ? 0 : count;

        neck_msg.data.push_back(tiltAngles_[count]);
        neck_msg.data.push_back(panAngles_[count]);

//        body_msg.data = body;
        neck_loop_.publish(neck_msg);
//        body_loop_.publish(body_msg);

        ROS_INFO("PUBLISHING ANGLE %d",tiltAngles_[count]);
        spinnerRate.sleep();
        ROS_INFO("DONE WAITING.....");

        count++;
    }
    ROS_INFO("STOPING IDLING LOOP.....");
    spinner.stop();
}

void HubertBrain::trackingStateLoop(){
    ros::AsyncSpinner spinner(1);
    ros::Rate spinnerRate(0.1);
    
    if(face_found) {
        std_msgs::Bool hello_msg;
        hello_msg.data = true;
        say_hello_.publish(hello_msg);
    }
    
    spinner.start();
    spinnerRate.sleep();

    if(face_found && ppl_passby_count == 0){
        std_msgs::Bool interrogateMsg;
        interrogateMsg.data = true;
        start_interrogation_.publish(interrogateMsg);
        robotState = RobotState::Interrogation;
    }
    else if (face_found && ppl_passby_count > 0) {
        ppl_passby_count = 0;
    }

    spinner.stop();
}


void HubertBrain::checkRobotStates(){
    if(robotState == RobotState::Idling){
        idlingLoop();
    }
    else if(robotState == RobotState::Tracking){
        trackingStateLoop();
    }
    else if(robotState == RobotState::Interrogation){
        //open treminal application
    }
    else{

    }

}



