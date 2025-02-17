#include "face_detection.h"

ROSFaceDetection::ROSFaceDetection():
 it_(nh_)
{
    std::string pkg_ = "face_detection";
    std::string ros_path_ = ros::package::getPath("face_detection");
    ros_path_ += "/shape_predictor_68_face_landmarks.dat";

    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ROSFaceDetection::imageCallBack,this);
    image_pub_ = it_.advertise("/face_detection/output_video", 1);

    img_location_ = nh_.advertise<face_detection::Face>("face_detection/img_location", 1);
    move_base_pub_ = nh_.advertise<face_detection::MoveBase>("face_detection/move_base",1);
    face_found_pub_ = nh_.advertise<std_msgs::Bool>("face_detection/face_found",1);

    ros::param::get("~SKIP_FRAMES",params["SKIP_FRAMES"]);
    ros::param::get("~RANGE_FOR_DETECTED", params["RANGE_FOR_DETECTED"]);
    ros::param::get("~RANGE_FOR_TRACKING", params["RANGE_FOR_TRACKING"]);

    faceTracker.initialize(ros_path_,params);
}

void ROSFaceDetection::execute(){
    ROS_INFO("START FACE TRACKING NODE !!!!");

    ros::Rate rate(10);
    while (ros::ok()) {
       getEllipseCenter();
       moveBase();
       faceFound();
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

    if(count_ % params["SKIP_FRAMES"] == 0){
        faceTracker.trackLandmark(cv_ptr->image,output_image_);
        cv_ptr->image = output_image_;
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    else image_pub_.publish(cv_ptr->toImageMsg());
}

void ROSFaceDetection::getEllipseCenter(){
    cv::RotatedRect minEllipse = faceTracker.requestEllipseCenter();
    face_detection::Face img_location;

    img_location.point.point.x = minEllipse.center.x;
    img_location.point.point.y = minEllipse.center.y;

    img_location.height.data = minEllipse.size.height;
    img_location.width.data = minEllipse.size.width;

    img_location_.publish(img_location);
}

void ROSFaceDetection::moveBase(){
    std::map<std::__cxx11::string,bool> move_base_data;
    move_base_data = faceTracker.moveBase();
    //if move base is true we publish it until it become false
    if(move_base_data["MOVE_BASE"] ){
        face_detection::MoveBase moveBaseMsg;
        moveBaseMsg.move_base = move_base_data["MOVE_BASE"];
        moveBaseMsg.turn_left = move_base_data["TURN_LEFT"];
        move_base_pub_.publish(moveBaseMsg);
    }
}

void ROSFaceDetection::faceFound(){
    std_msgs::Bool faceFoundMsg;
    faceFoundMsg.data = faceTracker.requestDetectedRealTime();
    face_found_pub_.publish(faceFoundMsg);
}