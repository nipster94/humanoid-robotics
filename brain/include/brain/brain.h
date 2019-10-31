/*!
 * @file brain.h
 * @brief This is the header file of the brain.cpp, which
 * was implemented to handle all the operation in the Hubert
 * system. This works as the centralized hub to control all t
 * the other nodes. The face detection/tracking, terminal
 * application and the Agent application.
 *
 * @author Fredrik Lagerstedt
 * @author Zhanyu Tuo
 * @author Terje Stenstrom
 * @author Nipun C. Gammanage
 *
 * @date Initial release - October/2019
*/
#ifndef BRAIN_H
#define BRAIN_H

#include "ros/ros.h"
#include "ros/package.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include "brain/RequestTerminal.h"
#include "brain/Access.h"
#include "brain/Feedback.h"

/**
 * \class ROSFaceDetection
 * \brief The ROS wrapper class for Hubert's Face Detection application
 *
 * Perform face detection and face tracking with the help of an
 * external library FaceTracking under ROS environment
 *
 */
class HubertBrain
{
    public:
        /**
         * \brief A Constructor for the class
         *
         *  Will be using to initialize initial values of
         *  the user variables and the ROS parameters,
         *  i.e Subscribers, Publishers and the lanuch file params
        */
        HubertBrain();

        /** \brief ENUM Defining tracking state available*/
        enum RobotState{
            Idling,                   /*!< Initial state - Contains the Idling loop */
            Tracking,                 /*!< Tracking state - Will move to this when a face is detected */
            Interrogation,            /*!< Interrogation state - Will handle opening the terminal application */
            FeedbackWaitingState,     /*!< Feedback waiting - To handle the waiting after interrogation */
            DecisionMaking            /*!< DecisionMaking  - Make the decision based on the interrogation output */
        };

        /** \brief ENUM Defining warning states available*/
        enum WarningState{
            NoWarning,                /*!< No warning - In the first 3 robots state the warningState is set to this */
            InitialWarning,           /*!< Initial warning - Define the initial warning after interrogation */
            SecondWarning,            /*!< Second warning */
            FinalWarning              /*!< Final warning */
        };
        /**
         * \fn void execute(void)
         * \brief The executor node
         *
         *  This node will be call from the ROS NODE main and will
         *  start the main ROS thread for publishers and subscribers.
         *  Node will run with 10 Hz rate.
         */
        void execute(void);

    private:
        /**
         * @defgroup  group1 ROS related variables
         * In this group all the variables related to ROS will be define
         * @{
         */
        ros::NodeHandle nh_;                        /**< ROS node handler */
        ros::Subscriber face_found_sub;             /**< This is the subscriber for the face found value. (From face_detection node) */
        ros::Subscriber feedback_sub;               /**< This is the subscriber for the feedback message. (From Agent node) */
        ros::Publisher neck_pan_loop_;              /**< Publish pan angle for the neck (To Servo/ Type UInt16) */
        ros::Publisher neck_tilt_loop_;             /**< Publish tilt angle for the neck (To Servo/ Type UInt16) */
        ros::Publisher body_loop_;                  /**< Publish base angle for the base of the robot (To Servo/ Type UInt16) */
        ros::Publisher robot_elbow_;                /**< Publish elbow angle for the arm of the robot (To Servo/ Type UInt16) */
        ros::Publisher robot_shoulder_;             /**< Publish shoulder angle for the arm of the robot (To Servo/ Type UInt16) */
        ros::Publisher fire_gun_;                   /**< Publish the fire gun command (To Servo/ Type Bool) */
        ros::Publisher say_hello_;                  /**< Publish the initial welcome command (To Agent/ Type Bool) */
        ros::Publisher start_interrogation_;        /**< Publish the interrogation dialogue trigger command (To Agent/ Type Bool) */
        ros::Publisher access_details_;             /**< Publish the output from the interrogation (To Agent/ Type brain::Access) */
	    ros::Publisher feedback_pub_;               /**< Publish the current status of the system  (To Agent/ Type brain::Feedback) */
        ros::ServiceClient terminalClient;          /**< ROS Service to handle the terminal application */
        /** @} */

        /**
         * @defgroup group2 General variables
         * In this group all the general variable will be define
         * @{
         */
        int pan_ub;                                             /**< Upper bound for the pan angle (user define from launch file) */
        int pan_lb;                                             /**< Lower bound for the pan angle (user define from launch file) */
        int pan_step_size;                                      /**< Step size for the pan angle (user define from launch file) */
        int tilt_ub;                                            /**< Upper bound for the tilt angle (user define from launch file) */
        int tilt_lb;                                            /**< Lower bound for the tilt angle (user define from launch file) */
        int tilt_step_size;                                     /**< Step size for the tilt angle (user define from launch file) */
        int body_ub;                                            /**< Upper bound for the move base angle (user define from launch file) */
        int body_lb;                                            /**< Lower bound for the move base angle (user define from launch file) */
        int body_step_size;                                     /**< Step size for the move base angle (user define from launch file) */
        int ppl_passby_count;                                   /**< The counter to check if the current tracking person disappear or not
                                                                  * (if the counter is zero then its the same person, else few people
                                                                  * have already pass the robot, when the robot in a certain transition state) */
        int current_body_angle;                                 /**< Track the current body angle. (The move base angle) */
        bool face_found;                                        /**< Boolean to represent the face found status */
        bool previous_face_found;                               /**< Detect the changes in the face found (current face disappear or not)*/
        bool state_changed;                                     /**< RobotState has change or not */
        bool access_granted;                                    /**< Decision variable for FeedbackWaitingState and DecisionMaking state */
        bool face_init;                                         /**< Initial state of the face detection/tracking */
        std::vector<uint> panAngles_;                           /**< Calculated pan angle sequence according to the parameters provided by the launch file */
        std::vector<uint> tiltAngles_;                          /**< Calculated tilt angle sequence according to the parameters provided by the launch file  */
        std::vector<uint> bodyAngles_;                          /**< Calculated body angle sequence according to the parameters provided by the launch file  */
        RobotState robotState = RobotState::Idling;             /**< Initial RobotState */
        WarningState warningState = WarningState::NoWarning;    /**< Initial WarningState */
        WarningState previousWS = WarningState::NoWarning;      /**< To keep track of the Warning State */
        /** @} */

        /**
         * \fn void faceFoundCallback(const std_msgs::Bool &msg)
         * \brief This callback will trigger with the face found details
         *
         * In this callback, the data coming from the face_detection
         * node will be analyzed. According to this boolean data
         * message the RobotState may change.
         */
        void faceFoundCallback(const std_msgs::Bool &msg);
        /**
         * \fn void feedbackCallback(const brain::Feedback &msg)
         * \brief This callback will trigger with the face found details
         *
         * In this callback, the data coming from the face_detection
         * node will be analyzed. According to this boolean data
         * message the RobotState may change.
         */
        void feedbackCallback(const brain::Feedback &msg);
        /**
         * \fn void checkRobotStates()
         * \brief This function will be used to switch the RobotState
         *
         * In This function, it will be constantly check the
         * current robot state. This will run in the main thread of the
         * ROS node
         */
        void checkRobotStates();
        /**
         * \fn void idlingLoop()
         * \brief This function will be used to perform the idling state
         *
         * In This function, it will publish data to servo package to
         * move the neck and body in a sequence. This will have the
         * access to the global variables panAngles_, tiltAngles_ and
         * bodyAngles_. Will run in the a different thread so that it
         * will not interrupt the main ROS node
         */
        void idlingLoop();
        /**
         * \fn void trackingStateLoop()
         * \brief This function will be used to perform face tracking
         *
         * In This function, face tracking will be performed. This
         * function will send commands to the trackFace node to perform
         * its task. Once the waiting is over the function will decide
         * whether or not to change to the interrogation state.
         * Will run in the a different thread so that it will not
         * interrupt the main ROS node
         */
        void trackingStateLoop();
        /**
         * \fn void handleInterrogation()
         * \brief This function will be used control the terminal application
         *
         * In This function, interrogation will be carried out. This
         * will handle the servo movement and the terminal application.
         * Upon the terminal application output the function will the
         * details to Agent. Will also initialize the tracking
         */
        void handleInterrogation();
        /**
         * \fn void feedbackWaiting()
         * \brief This function handle the waiting times
         *
         * In This function, the waiting time between the Agent and the
         * current robot status will be handled. The state FeedbackWaiting
         * was introduced to handle the the transition section from
         * Interrogation to DecisionMaking, as there will be few sub
         * processes running in the back-ground.
         */
        void feedbackWaiting();
        /**
         * \fn void feedbackWaiting()
         * \brief This function handles the decision making state
         *
         * In This function, DecisionMaking state will be handle.
         * The state DecisionMaking will be responsible to handle the
         * last phase of the operation. Upon the decision the robot
         * will publish date to the servo node or it will simply do a
         * reset
         */
        void takeDecision();
        /**
         * \fn void resetSystem()
         * \brief This function handles the resetting process
         *
         * In This function, the system will be resetted. All the
         * decision variables will be set to the initial values and
         * the robot will also move to its initial positions
         */
        void resetSystem();

        /**
         * \fn std::vector<uint> getPanAngles(int lb, int ub, int stepSize)
         * \brief Calculated pan angle sequence
         *
         * The values will be according to the parameters provided by the
         * launch file
         * \param [in] lb Lower bound
         * \param [in] ub Upper bound
         * \param [in] stepSize step size of the pan angle
         * \return A vector with pan angles that should be published
         *         in the idling loop
         */
        std::vector<uint> getPanAngles(int lb, int ub, int stepSize);
        /**
         * \fn std::vector<uint> getTiltAngles(int lb, int ub, int stepSize)
         * \brief Calculated tilt angle sequence
         *
         * The values will be according to the parameters provided by the
         * launch file
         * \param [in] lb Lower bound
         * \param [in] ub Upper bound
         * \param [in] stepSize step size of the tilt angle
         * \return A vector with pan angles that should be published
         *         in the idling loop
         */
        std::vector<uint> getTiltAngles(int lb, int ub, int stepSize);
        /**
         * \fn std::vector<uint> getBodyAngles(int lb, int ub, int stepSize)
         * \brief Calculated body angle sequence
         *
         * The values will be according to the parameters provided by the
         * launch file
         * \param [in] lb Lower bound
         * \param [in] ub Upper bound
         * \param [in] stepSize step size of the angle
         * \return A vector with body angles that should be published
         *         in the idling loop
         */
        std::vector<uint> getBodyAngles(int lb, int ub, int stepSize);
};


#endif
