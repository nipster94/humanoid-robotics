/*!
* @file face_tracking.h
* @brief This is the header file of the face_tracking.cpp, which
* was implemented to detect faces, track the closest face. This
* method is also called as the First Landmark Tracking, as it will
* focus only on the first person who enters the screen or if many
* enters the frame then the closest person (by analysing the area of
* the bounding box)
*
* @author Fredrik Lagerstedt
* @author Zhanyu Tuo
* @author Terje Stenstrom
* @author Nipun C. Gammanage
*
* @date Initial release - October/2019
*/
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

/**
* \class FaceTracking
* \brief A library to handle face detection tracking and
* face tracking
*
* Perform face detection and face tracking.
*
*/
class FaceTracking
{
    public:
        /**
         * \brief A Constructor for the class
         *
         *  Will be using to initialize initial values of
         *  the user variables
        */
        FaceTracking();
        /**
         * \brief A Destructor
         *
         * This is an empty destructor created for future use (can remove)
        */
        ~FaceTracking(){}

        /*! This is an enum class to check the current tracking state*/
        enum TrackingState{
             Initialize     /*!< Initial state */,
             Tracking       /*!< Tracking state */,
             Updating       /*!< Update ground truth */
        };

        /**
         * \brief Initialize the library
         * This is the function which initialize the library and this must
         * be called otherwise, the other function cannot run this library.
         * The ROS parameter which are defined in launch files will be passed
         * into the library via this function
         * \param [in] path_ path to the dlib pose model
         * \param [in] param_ ROS params
         */
        void initialize(std::string path_ , std::map<std::__cxx11::string, int> &param_);

        /**
         * \brief Face detection/tracking function
         * This is the main trigger function. Takes input image and returns the
         * detailed tracked image
         * \param [in] input_image_ input image
         *
         * \return output image
         */
        void trackLandmark(cv::Mat &input_image_, cv::Mat &output_image_);
        /**
         * \brief Center bound of ellipse of the current tracking face will
         * be returned, upon the request from the ROS node
         *
         * \return return the bound of ellipse of the current tracking face
         */
        cv::RotatedRect requestEllipseCenter();
        /**
         * \brief Status of the face detection, This value is a boolean
         * variable which return true if a face is present in the current
         * frame
         *
         * \return return the status of the face detection
         */
        bool requestDetectedRealTime();
        /**
         * \brief Number of close faces
         *
         * \return return the number of close faces detected
         */
        int requestFacesSize();
        /**
         * \brief Status of the base frame
         * This function will return whether to whether or not
         * to rotate the base of the robot. Furthermore it will
         * als return which way the robot should turn
         * Here move is true when the camera needs to rotate and
         * the direction tells whether to turn left or right
         *
         * \return return the move base command (move,direction)
         */
        std::map<std::__cxx11::string, bool> moveBase();
    int Temp();

    private:
        int cfacesize = 0;                          /**< Number of close faces detected */
        int RANGE_FOR_DETECTED;                     /**< Detection range */
        int RANGE_FOR_TRACKING;                     /**< Tracking range */
        bool face_found;                            /**< Detection status */
        bool move_base;                             /**< Base status */
        bool turnLeft;                              /**< Base direction value (true if left, false otherwise) */
        TrackingState state = Initialize;           /**< Tracking status */
        cv::RotatedRect minEllipse;                 /**< Tracked face ellipse */
        std::vector<cv::Point> ground_truth;        /**< Ground truth */
        dlib::frontal_face_detector detector;       /**< Dlib - face landmark detection */
        dlib::shape_predictor pose_model;           /**< Dlib - pose model for face landmark detection */

        /**
         * \fn float calculateArea(float semi_major_axis, float semi_minor_axis)
         * \brief calculate the area of the ellipse
         * \param [in] semi_major_axis ellipse width
         * \param [in] semi_minor_axis ellipse height
         * \return return the area of the ellipse (float)
         */
        float calculateArea(float semi_major_axis, float semi_minor_axis);
        /**
         * \fn RMSE( std::vector<cv::Point> ground_truth, std::vector<cv::Point> fitted_shapes)
         * \brief calculate Root Mean Square Error of the current face and the ground truth
         * here the fround truth will be the values of the previously tracked face
         * \param [in] ground_truth ground truth face (vector point of the ellipse)
         * \param [in] fitted_shapes currently tracked face (vector point of the ellipse)
         * \return return the results from the calculation (float)
         */
        float RMSE( std::vector<cv::Point> ground_truth, std::vector<cv::Point> fitted_shapes);
        /**
         * \fn getTrackingIndex(dlib::cv_image<dlib::bgr_pixel> img_, std::vector<dlib::rectangle> faces_)
         * \brief return tracking index of the current tracking face of the dlib face rectangle vector
         * \param [in] img_ input image
         * \param [in] faces_ dlib face rectangle vector
         * \return index of the current tracking face
         */
        int getTrackingIndex(dlib::cv_image<dlib::bgr_pixel> img_, std::vector<dlib::rectangle> faces_);
        /**
         * \fn getCloseFaces(std::vector<dlib::rectangle> faces_,int tracking_index)
         * \brief Choose whether the face is in the correct range. We have provided
         * some parameters which could decide the range of tracking.
         * \param [in] faces_ dlib face rectangle vector
         * \param [in] tracking_index current tracking index
         * \return return dlib face rectangle vector with the new close face
         * \remark
         *      - Range of tracking  : Upto what extend should we keep tracking this person
         *      - Range of detection : Upto what extend should we consider detecting a person
         *      - tracking index
         */
        std::vector<dlib::rectangle> getCloseFaces(std::vector<dlib::rectangle> faces_,int tracking_index);
        /**
         * \fn getMaxAreaIndex(dlib::cv_image<dlib::bgr_pixel> &img_,
                             std::vector<dlib::rectangle> &input,
                             cv::vector<cv::vector<cv::Point> > &all_landmarks_,
                             cv::vector< cv::vector<cv::Point> > &jaw_line,
                             int &max_area_index);
         * \brief calculate the area of the ellipse and to find which face has the largest area
         *        (closest to the camera)
         * \param [in] img_ input image ni the dlib format
         * \param [in] input dlib face rectangle vector
         * \param [out] all_landmarks_ get the points of all faces deteced by the dlib
         *              face detector
         * \param [out] jaw_line get the jaw line points (from the 68 points) of all
         *              faces detected by the dlib face detector
         * \param [out] max_area_index find the face with the maximum area
         * \remark
         *      - The jaw line points will be used to form an ellipse, which will then
         *        be use to calculate the area
         */
        void getMaxAreaIndex(dlib::cv_image<dlib::bgr_pixel> &img_,
                             std::vector<dlib::rectangle> &input,
                             cv::vector<cv::vector<cv::Point>> &all_landmarks_,
                             cv::vector<cv::vector<cv::Point>> &jaw_line,
                             int &max_area_index);
        /**
         * \fn startTracking(cv::Mat &image_,
                             std::vector<dlib::rectangle> &cfaces,
                             cv::vector<cv::vector<cv::Point> > &all_landmarks_,
                             cv::vector< cv::vector<cv::Point> > &jaw_line,
                             int &max_area_index);
         * \brief calculate the area of the ellipse and to find which face has the largest area
         *        (closest to the camera)
         * \param [in] img_ input image in cv::Mat format
         * \param [in] cfaces dlib face rectangle vector of the close face
         * \param [in] all_landmarks_ get the points of all faces deteced by the dlib
         *             face detector
         * \param [in] jaw_line get the jaw line points (from the 68 points) of all
         *             faces detected by the dlib face detector
         * \param [in] max_area_index find the face with the maximum area
         * \remark
         *      - The input image will be over written withe the new information about the faces
         *      - This function also perform the tracking part in this library
         */
        void startTracking(cv::Mat &image_,
                           std::vector<dlib::rectangle> &cfaces,
                           cv::vector<cv::vector<cv::Point>> &all_landmarks_,
                           cv::vector<cv::vector<cv::Point>> &jaw_line,
                           int &max_area_index);
};

#endif
