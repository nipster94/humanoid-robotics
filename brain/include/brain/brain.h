#ifndef BRAIN_H
#define BRAIN_H

#include "ros/ros.h"
#include "ros/package.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include "brain/RequestTreminal.h"
#include "brain/Access.h"
#include "brain/Feedback.h"

class HubertBrain
{
    public:
        HubertBrain();

        enum RobotState{
            Idling,
            Tracking,
            Interrogation,
            FeedbackWaitingState,
            DecisionMaking
        };

        enum WarningState{
            NoWarning,
            InitialWarning,
            SecondWarning,
            FinalWarning
        };

        void execute(void);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle temp_nh_;
        ros::Subscriber face_found_sub;
        ros::Subscriber feedback_sub;
//        boost::shared_ptr<ros::AsyncSpinner> global_spinner;
//        ros::AsyncSpinner global_spinner;
//        ros::CallbackQueue* queue;

        ros::Publisher neck_pan_loop_;
        ros::Publisher neck_tilt_loop_;
        ros::Publisher body_loop_;
        ros::Publisher robot_elbow_;
        ros::Publisher robot_shoulder_;
        ros::Publisher fire_gun_;

        ros::Publisher say_hello_;
        ros::Publisher start_interrogation_;
        ros::Publisher access_details_;
	    ros::Publisher feedback_pub_;
        

        ros::ServiceClient terminalClient;

        int pan_ub;
        int pan_lb;
        int pan_step_size;
        int tilt_ub;
        int tilt_lb;
        int tilt_step_size;
        int body_ub;
        int body_lb;
        int body_step_size;

        int ppl_passby_count;

        std::vector<uint> panAngles_;
        std::vector<uint> tiltAngles_;
        std::vector<uint> bodyAngles_;

        RobotState robotState = RobotState::Idling;
        WarningState warningState = WarningState::NoWarning;

        bool face_found;
        bool previous_face_found;
        bool state_changed;
        bool access_granted;
        bool face_init;

        void checkRobotStates();
        void idlingLoop();
        void trackingStateLoop();
        void handleInterrogation();
        void feedbackWaiting();
        void takeDecision();

        void faceFoundCallback(const std_msgs::Bool &msg);
        void feedbackCallback(const brain::Feedback &msg);

//        std::list<uint> getPanTiltAngles(int lb, int ub, int stepSize);
        std::vector<uint> getPanAngles(int lb, int ub, int stepSize);
        std::vector<uint> getTiltAngles(int lb, int ub, int stepSize);
        std::vector<uint> getBodyAngles(int lb, int ub, int stepSize);


};


#endif
