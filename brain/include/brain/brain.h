#ifndef BRAIN_H
#define BRAIN_H

#include "ros/ros.h"
#include "ros/package.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

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
        ros::Subscriber face_found_sub;

        RobotState robotState = Idling;
        bool face_found;
        bool previous_face_found;

        void checkRobotStates();

        void faceFoundCallback(const std_msgs::Bool &msg);

};


#endif
