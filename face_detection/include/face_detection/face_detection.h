/*!
 * @file face_detection.h
 * @brief This is the header file of the face_detection/_node.cpp, which
 * was implemented as a ROS wrapper. This ROS node will use the
 * face detection library to perform face detection and face tracking.
 * The tracked face details will then be published to the related topics
 * so that it will grantee a smooth process in the Hubert Brain
 *
 * @author Fredrik Lagerstedt
 * @author Zhanyu Tuo
 * @author Terje Stenstrom
 * @author Nipun C. Gammanage
 *
 * @date Initial release - October/2019
*/

#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include "ros/ros.h"
#include "ros/package.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "face_tracking.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>        //Converting ROS to OpenCV images
#include <cv_bridge/cv_bridge.h>                //Converting ROS to OpenCV images
#include <image_transport/image_transport.h>    //Publishing and subscribing to images in ROS
#include <opencv2/imgproc/imgproc.hpp>          //Converting ROS to OpenCV images
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include "face_detection/MoveBase.h"
#include "face_detection/Face.h"

/**
 * \class ROSFaceDetection
 * \brief The ROS wrapper class for Hubert's Face Detection application
 *
 * Perform face detection and face tracking with the help of an
 * external library FaceTracking under ROS environment
 *
 */
class ROSFaceDetection
{
    public:
        FaceTracking faceTracker;
        /**
         * \brief A Constructor for the class
         *
         *  Will be using to initialize initial values of
         *  the user variables and the ROS parameters,
         *  i.e Subscribers, Publishers and the lanuch file params
        */
        ROSFaceDetection();
        /**
         * \brief The executor node
         *
         *  This node will be call from the ROS NODE main and will
         *  start the main ROS thread for publishers and subscribers.
         *  Node will run with 10 Hz rate.
         */
        void execute(void);

    private:
        /**
         * @defgroup  group1 ROS related varialbles
         * In this group all the variables related to ROS will be define
         * @{
         */
        ros::NodeHandle nh_;                        /**< ROS node handler */
        image_transport::ImageTransport it_;        /**< This is used to subscribe to and publish images. In other words, this work as a node handler for images */
        image_transport::Subscriber image_sub_;     /**< This is the image subscriber(will get subscribe from the camera) */
        image_transport::Publisher image_pub_;      /**< This is the image publisher(will publish to the system) */
        ros::Publisher img_location_;               /**< Publish the center points of the ellipse and its height and width Face.msg type data */
        ros::Publisher move_base_pub_;              /**< Publish MoveBase details MoveBse.msg type */
        ros::Publisher face_found_pub_;             /**< Publish Face detection status (False - Not detected True - Detected)*/
        cv_bridge::CvImagePtr cv_ptr;               /**< Image pointer */
        /** @} */

        /**
         * @defgroup group2 General variables
         * In this group all the general variable will be define
         * @{
         */
        std::map<std::string , int> params;         /**< ROS parameter std::map<string,int> use to pass the variables to the library */
        int count_ = 0;                             /**< Counter to track the skip_frame parameter and to execute */
        bool currentMoveBase = true;
        int skip_val;
        /** @} */

        /**
         * \brief This callback will grab the image data from the camera
         * \param msg sensor_msgs::ImageConstPtr type message
         *
         * In this callback the ROS image data will be converted to
         * OpeCV image(cv::Mat) in order to perform the further processing
         * \remark
         *     - The image msg will be converted via cv_bridge
         *       \link http://docs.ros.org/jade/api/cv_bridge/html/c++/cv__bridge_8h.html
         *       \endlink
         *     - The tracked image, with the details will be published
         */
        void imageCallBack( const sensor_msgs::ImageConstPtr& msg);
        /**
         * \brief This callback will publish the detected face details
         *
         * In this callback, center points of the ellipse center
         * along with its height and the width of the bounding box of the
         * detected face will be published to the topic,
         * /face_detection/img_location. The data will be in the type of
         * FaceDetection::Face
         */
        void getEllipseCenter();
        /**
         * \brief This callback will publish move base status
         *
         * In this callback, the status of the MoveBase wil be published.
         * This will tell its subscribers to whether to move the base
         * or not and to what direction. It will be publish to the topic
         * face_detection/move_base. The data will be in the type of
         * face_detection::MoveBase
         */
        void moveBase();
        /**
         * \brief This callback will publish the current status of face detection
         *
         * In this callback, the status of the tracking will be published, via the topic
         * "/face_detection/face_found". The function will be checked in
         * every iteration
         *
         * \remark
         *     - detected     --> True
         *     - not detected --> False
         */
        void faceFound();

};

#endif
