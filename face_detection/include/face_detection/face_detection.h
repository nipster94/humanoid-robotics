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

class ROSFaceDetection
{
    public:
        FaceTracking faceTracker;

        ROSFaceDetection();

        void execute(void);

    private:
        int count_ = 0;
        bool currentMoveBase = true;
        ros::NodeHandle nh_;                        /**< ROS node handler */
        geometry_msgs::PointStamped img_points;

        image_transport::ImageTransport it_;        /**< This is used to subscribe to and publish images. In other words, this work as a node handler for images */

        image_transport::Subscriber image_sub_;     /**< This is the image subscriber(will get subscribe from the camera) */
        image_transport::Publisher image_pub_;      /**< This is the image publisher(will publish to the system) */

        ros::Publisher debug_msg_pub_;
        ros::Publisher img_points_;
        ros::Publisher move_base_pub_;
        ros::Publisher face_found_pub_;
        geometry_msgs::PointStamped valPoints;      /**< Point x and y of the ellipse center */
        std_msgs::Int32 detection_flag;             /**< TODO */
        std_msgs::Int32 detection_flag_real_time;   /**< Status of the real time face detection (0 - Not detected 1 - Detected) */
        cv_bridge::CvImagePtr cv_ptr;               /**< Image pointer */

        std::map<std::string , int> params;

        int skip_val;

        void imageCallBack( const sensor_msgs::ImageConstPtr& msg);

        void getEllipseCenter();

        void moveBase();

        void faceFound();

};

#endif
