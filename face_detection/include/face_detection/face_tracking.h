#ifndef FACE_TRACKING_H
#define FACE_TRACKING_H

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>

namespace cv
{
    using std::vector;
}

class FaceTracking
{
    public:
        FaceTracking();
        ~FaceTracking(){}

        /*! This is an enum class to check the current tracking state*/
        enum TrackingState{
             Initialize /*!< Initial state */,
             Tracking /*!< Tracking state */,
             Updating /*!< Update ground truth */
        };

//        void initialize(std::string path_,
//                        std::map<std::string , int> &param_ );

        void initialize(std::string path_ );

        void trackLandmark(cv::Mat &input_image_, cv::Mat &output_image_, std::string message);

        cv::RotatedRect requestEllipseCenter();
        bool requestDetectedRealTime();
        int requestFacesSize();

    private:

        int cfacesize = 0;                          /**< Number of close faces detected */
        int CHECK_PERSON_DETECTED;                  /**< TODO */
        int CHECK_PERSON_DISAPPEAR;                 /**< TODO */
        int RANGE_FOR_DETECTED;                     /**< Detecetion range */
        int RANGE_FOR_TRACKING;                     /**< Tracking range */
        bool face_found;                            /**< Detection status */
        TrackingState state = Initialize;           /**< Tracking status */
        cv::RotatedRect minEllipse;                 /**< Tracked face ellipse */
        std::vector<cv::Point> ground_truth;        /**< Ground truth */
        dlib::frontal_face_detector detector;       /**< Dlib - face landmark detection */
        dlib::shape_predictor pose_model;           /**< Dlib - pose model for face landmark detection */

        float calculateArea(float semi_major_axis, float semi_minor_axis);
        float RMSE( std::vector<cv::Point> ground_truth, std::vector<cv::Point> fitted_shapes);
        int getTrackingIndex(dlib::cv_image<dlib::bgr_pixel> img_, std::vector<dlib::rectangle> faces_);
        std::vector<dlib::rectangle> getCloseFaces(std::vector<dlib::rectangle> faces_,int tracking_index);
        void getMaxAreaIndex(dlib::cv_image<dlib::bgr_pixel> &img_,
                             std::vector<dlib::rectangle> &input,
                             cv::vector<cv::vector<cv::Point>> &all_landmarks_,
                             cv::vector<cv::vector<cv::Point>> &jaw_line,
                             int &max_area_index);
        void startTracking(cv::Mat &image_,
                           std::vector<dlib::rectangle> &cfaces,
                           cv::vector<cv::vector<cv::Point>> &all_landmarks_,
                           cv::vector<cv::vector<cv::Point>> &jaw_line,
                           int &max_area_index);

};

#endif
