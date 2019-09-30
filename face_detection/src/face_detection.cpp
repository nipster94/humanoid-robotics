#include "face_detection.h"

ROSFaceDetection::ROSFaceDetection():
 it_(nh_)
{
    std::string pkg_ = "face_detection";
//    std::string ros_path_ = ros::package::getPath("face_detection_ws");
//    ros_path_.erase(ros_path_.find(pkg_), pkg_.size());
//    std::string ros_path_ = "/home/nipun/MPSYS/Q5/Humanoid_Robotics/Project/face_detection_ws/src/face_detection/shape_predictor_68_face_landmarks.dat";
    std::string ros_path_ = "/home/nipun/MPSYS/Q5/Humanoid_Robotics/Project/face_detection_ws/src/face_detection/shape_predictor_5_face_landmarks.dat";

    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ROSFaceDetection::imageCallBack,this);

    image_pub_ = it_.advertise("/face_tracking/output_video", 1);

    debug_msg_pub_ = nh_.advertise<std_msgs::String>("debug_msg", 1000);

    skip_val = 1;

    faceTracker.initialize(ros_path_);
}

void ROSFaceDetection::execute(){
    ROS_INFO("START FACE TRACKING NODE !!!!");
    ros::Rate rate(10);
    while (ros::ok()) {
       count_++;
       if (count_ > 1000) count_ = 0;
       rate.sleep();
       ros::spinOnce();

    }
}

void ROSFaceDetection::imageCallBack(const sensor_msgs::ImageConstPtr &msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat output_image_;
    std::string message;

    faceTracker.trackLandmark(cv_ptr->image,output_image_,message);
    cv_ptr->image = output_image_;
    image_pub_.publish(cv_ptr->toImageMsg());

//    output_image_ = cv_ptr->image;

//    char str[200];
//    sprintf( str, "TRACKING");
//    putText( output_image_, str, cv::Point(output_image_.rows/2,output_image_.cols/2), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 3, 8, false );

//    cv_ptr->image = output_image_;

//    image_pub_.publish(cv_ptr->toImageMsg());

//    cv::imshow( "Frame", output_image_ );


//    cv_ptr->image = output_image_;
//    image_pub_.publish(cv_ptr->toImageMsg());


//     if (count_ % skip_val == 0 ) {
//         faceTracker.trackLandmark(cv_ptr->image,output_image_,message);
//         cv_ptr->image = output_image_;
//         image_pub_.publish(cv_ptr->toImageMsg());
//     }
//     else image_pub_.publish(cv_ptr->toImageMsg());

//    faceTracker.trackLandmark(cv_ptr->image,output_image_,message);
//    cv_ptr->image = output_image_;
//    image_pub_.publish(cv_ptr->toImageMsg());

    std_msgs::String debug_msg;
    debug_msg.data = message;

    debug_msg_pub_.publish(debug_msg);
}

