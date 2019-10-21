#include "brain.h"

HubertBrain::HubertBrain():
    nh_("~")
{
    face_found = false;
    previous_face_found = false;
    face_init = true;
    state_changed = false;

//    neck_loop_= nh_.advertise<std_msgs::UInt16MultiArray>("/hubert_brain/servo_neck",1);
//    body_loop_ = nh_.advertise<std_msgs::UInt16>("/hubert_brain/servo_body",1);

    neck_pan_loop_= nh_.advertise<std_msgs::UInt16>("/servo_neck_rot",1);
    neck_tilt_loop_= nh_.advertise<std_msgs::UInt16>("/servo_neck_tilt",1);
    body_loop_ = nh_.advertise<std_msgs::UInt16>("/servo_body",1);    
    robot_elbow_ = nh_.advertise<std_msgs::UInt16>("/servo_elbow",1);
    robot_shoulder_ = nh_.advertise<std_msgs::UInt16>("/servo_shoulder",1);

    fire_gun_ = nh_.advertise<std_msgs::Bool>("/fire_gun",1);

    say_hello_ = nh_.advertise<std_msgs::Bool>("/hubert_brain/say_hello",1);
    start_interrogation_ = nh_.advertise<std_msgs::Bool>("/hubert_brain/start_interrogation",1);
    access_details_ = nh_.advertise<brain::Access>("/hubert_brain/access_details",1);

    feedback_pub_ = nh_.advertise<brain::Feedback>("/hubert_brain/feedback",1);
    
    face_found_sub = nh_.subscribe("/face_detection/face_found", 1, &HubertBrain::faceFoundCallback,this);
    feedback_sub = nh_.subscribe("/hubert_brain/feedback",1,&HubertBrain::feedbackCallback,this);

    terminalClient = nh_.serviceClient<brain::RequestTreminal>("/hubert_brain/terminal");


    ros::param::get("~PAN_LB",pan_lb);
    ros::param::get("~PAN_UB", pan_ub);
    ros::param::get("~PAN_STEP_SIZE", pan_step_size);

    ros::param::get("~TILT_LB",tilt_lb);
    ros::param::get("~TILT_UB", tilt_ub);
    ros::param::get("~TILT_STEP_SIZE", tilt_step_size);

    ros::param::get("~BODY_LB",body_lb);
    ros::param::get("~BODY_UB", body_ub);
    ros::param::get("~BODY_STEP_SIZE", body_step_size);

    ppl_passby_count = 0;

}

void HubertBrain::execute(){
    ROS_INFO("STARTING HUBERT BRAIN!!!");
    ros::Rate rate(10);
    panAngles_ = getPanAngles(pan_lb,pan_ub,pan_step_size);
    tiltAngles_ = getTiltAngles(tilt_lb,tilt_ub,tilt_step_size);
    bodyAngles_ = getBodyAngles(body_lb,body_ub,body_step_size);
    while(ros::ok()){
        checkRobotStates();
        rate.sleep();
        ros::spinOnce();
    }
}

void HubertBrain::faceFoundCallback(const std_msgs::Bool &msg){
    face_found = msg.data;

    if(face_init && face_found){
        ROS_INFO("Initial state");
        previous_face_found = false;
        face_init = false;
        state_changed = false;
    }
    else if(face_found != previous_face_found){
        ROS_INFO("Face detection state changed");
        if(robotState == RobotState::Idling){
            robotState = RobotState::Tracking;
            state_changed = true;
        }
        else if (robotState == RobotState::Tracking && face_found){           
            ppl_passby_count += 1;
            ROS_INFO("PEOPLE FOUND %d",ppl_passby_count);
        }
        else if (robotState == RobotState::Interrogation && face_found) {
            ppl_passby_count = 0;
        }
        else if(robotState == RobotState::DecisionMaking && face_found){
            ppl_passby_count += 1;
        }

        previous_face_found = face_found;
    }
    else{
        if(face_found && robotState == RobotState::Tracking){

        }
        else if(face_found && robotState == RobotState::DecisionMaking){
            ROS_WARN("DETECTING THE SAME PERSON");
        }
        
    }
}

void HubertBrain::feedbackCallback(const brain::Feedback &msg){
//    std::cout << msg.agent_feedback << std::endl;

    ROS_DEBUG("got info %s",msg.agent_feedback.c_str());

    if(msg.agent_feedback == "initial welcome" ||
       msg.agent_feedback == "initial warning"){
        robotState = RobotState::DecisionMaking;
    }
    else if(msg.agent_feedback == "1st warning issued"){
        warningState = WarningState::InitialWarning;
    }
    else if (msg.agent_feedback == "2nd warning issued"){
        warningState = WarningState::SecondWarning;
    }
    else if (msg.agent_feedback == "3rd warning issued"){
        warningState = WarningState::FinalWarning;
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

std::vector<uint> HubertBrain::getBodyAngles(int lb, int ub, int stepSize){
    bool goingUp = true;
    bool goingDown = false;

    std::vector<uint> bodyAngles;
    int currentAngle = 90;
    bodyAngles.push_back(currentAngle);

    int vecSize = ((ub-lb)/stepSize)*2;

    std::cout << vecSize << std::endl;

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

        bodyAngles.push_back(currentAngle);
    }
	
    return bodyAngles;
}

void HubertBrain::idlingLoop(){
    int pan_count = 0;
    int tilt_count = 0;
    int body_count = 0;
    
    ros::AsyncSpinner spinner(1);
    ros::Rate spinnerRate(0.5);
    spinner.start();
    while(ros::ok() && !state_changed){
        std_msgs::UInt16 pan_msg;
        std_msgs::UInt16 tilt_msg;
        std_msgs::UInt16 body_msg;

        pan_count = pan_count == panAngles_.size() ? 0 : pan_count;
        tilt_count = tilt_count == tiltAngles_.size() ? 0 : tilt_count;
        body_count = body_count == bodyAngles_.size() ? 0 : body_count;

//        neck_msg.data.push_back(panAngles_[pan_count]);
//        neck_msg.data.push_back(tiltAngles_[tilt_count]);

//        body_msg.data = body;
//        neck_loop_.publish(neck_msg);
//        body_loop_.publish(body_msg);

        pan_msg.data = panAngles_[pan_count];
        tilt_msg.data = tiltAngles_[tilt_count];
        body_msg.data = bodyAngles_[body_count];

        neck_pan_loop_.publish(pan_msg);
        neck_tilt_loop_.publish(tilt_msg);
        body_loop_.publish(body_msg);

        ROS_INFO("PUBLISHING ANGLES %d, %d, %d", panAngles_[pan_count], tiltAngles_[tilt_count], bodyAngles_[body_count]);
        spinnerRate.sleep();
        ROS_INFO("DONE WAITING.....");

        pan_count++;
        tilt_count++;
        body_count++;
    }
    ROS_INFO("ENDING IDLING LOOP.....");
    spinner.stop();
}

void HubertBrain::trackingStateLoop(){
    ros::AsyncSpinner spinner(1);
    ros::Rate spinnerRate(0.1);

    ROS_INFO("IN THE TRACKING STATE");
    
    if(face_found) {
        std_msgs::Bool hello_msg;
        hello_msg.data = true;
        say_hello_.publish(hello_msg);
    }
    
    spinner.start();
    spinnerRate.sleep();
    spinner.stop();

    ROS_INFO("TRACKING FINISHED");

    if(face_found && ppl_passby_count == 0){
        std_msgs::Bool interrogateMsg;
        interrogateMsg.data = true;
        start_interrogation_.publish(interrogateMsg);
        robotState = RobotState::Interrogation;

        //Need to wait for few seconds until language processing
        //part finishes its sentence

        //ros::Duration(5).sleep();

    }
    else if (face_found && ppl_passby_count > 0) {
        ppl_passby_count = 0;
    }
    else if (!face_found) {
        robotState = RobotState::Idling;
        state_changed = false;
    }
}

void HubertBrain::handleInterrogation(){
    ROS_WARN("HANDLE INTERROGATION");

    std_msgs::UInt16 elbow;
    elbow.data = 60;
    robot_elbow_.publish(elbow);

    std_msgs::UInt16 shoulder;
    shoulder.data = 20;
    robot_shoulder_.publish(shoulder);

    brain::RequestTreminal terminalService;
    terminalService.request.open_terminal = true;

    if(terminalClient.call(terminalService)){
        brain::Access accessMsg;
        accessMsg.access_granted = terminalService.response.access;
        accessMsg.user_name = terminalService.response.name;
        access_details_.publish(accessMsg);
        access_granted = terminalService.response.access;
    }
    else{
         ROS_ERROR("Failed to call terminal service");
    }

    ros::Duration(5).sleep();

    robotState = RobotState::FeedbackWaitingState;
}

void HubertBrain::feedbackWaiting() {
    ROS_INFO("WAITING FOR FEEDBACK");

    std_msgs::UInt16 elbow;
    std_msgs::UInt16 shoulder;

    if(access_granted){
        elbow.data = 60;
        robot_elbow_.publish(elbow);

        shoulder.data = 90;
        robot_shoulder_.publish(shoulder);
    } else{
        //Move arm to shoot
        elbow.data = 40;
        robot_elbow_.publish(elbow);

        shoulder.data = 20;
        robot_shoulder_.publish(shoulder);
    }
}

void HubertBrain::takeDecision(){
    ROS_WARN("TAKING A DECISION");

    if(!access_granted) {
        //Give first warning via feedback msg
        brain::Feedback brainFeedback;
        brainFeedback.brain_feedback = "1 warning";
        feedback_pub_.publish(brainFeedback);

        ros::AsyncSpinner spinner(1);
        ros::Rate spinnerRate(0.5);
        bool exit_loop = false;
        
        spinner.start();
        //Wait after the first warning
        ros::Duration(5).sleep();
                
        while(!exit_loop){

            if(ppl_passby_count == 0 && face_found &&
                    warningState == WarningState::InitialWarning){
                //Give second warning via feedback msg
                brainFeedback.brain_feedback = "2 warning";
                feedback_pub_.publish(brainFeedback);
                ROS_INFO("2 warning");
            }
            else if(ppl_passby_count == 0 && face_found &&
                    warningState == WarningState::SecondWarning){
                brainFeedback.brain_feedback = "3 warning";
                feedback_pub_.publish(brainFeedback);
                ROS_INFO("3 warning");
            }
            else if(ppl_passby_count > 0 && face_found){
                exit_loop = true;
                ROS_INFO("face found");
            }
            else if(warningState == WarningState::FinalWarning){
                exit_loop = true;
                ROS_INFO("final warning");
            }

            spinnerRate.sleep();
        }

        if(ppl_passby_count == 0 and face_found){
            std_msgs::Bool fire_gun;
            fire_gun.data = true;
            fire_gun_.publish(fire_gun);
        }

        spinner.stop();

    }
    ROS_WARN("WAITING");
    ros::Duration(5).sleep();
    robotState = face_found ? RobotState::Idling : RobotState::Tracking;
    face_init = true;
}

void HubertBrain::checkRobotStates(){
    if(robotState == RobotState::Idling){
        idlingLoop();
    }
    else if(robotState == RobotState::Tracking){
        trackingStateLoop();
    }
    else if(robotState == RobotState::Interrogation){
        handleInterrogation();
    }
    else if(robotState == RobotState::FeedbackWaitingState){
        feedbackWaiting();
    }
    else if(robotState == RobotState::DecisionMaking){
        takeDecision();
    }
}


