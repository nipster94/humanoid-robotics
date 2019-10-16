#ifndef BRAIN_H
#define BRAIN_H

#include "ros/ros.h"
#include "ros/package.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>

class HubertBrain
{
    public:
        HubertBrain();

        enum RobotState{
            Idling,
            Tracking,
            Interrogation,
            DecisionMaking
        };

        void execute(void);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle temp_nh_;
        ros::Subscriber face_found_sub;
//        boost::shared_ptr<ros::AsyncSpinner> global_spinner;
//        ros::AsyncSpinner global_spinner;
//        ros::CallbackQueue* queue;

        ros::Publisher neck_loop_;
        ros::Publisher body_loop_;

        int pan_tilt_ub;
        int body_ub;
        int pan_tilt_lb;
        int body_lb;

        RobotState robotState = Idling;
        bool face_found;
        bool previous_face_found;
        bool state_changed;

        void checkRobotStates();
        void idlingLoop();

        void faceFoundCallback(const std_msgs::Bool &msg);

};


#endif
