//#include "face_tracking.h"

#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <X11/Xlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <math.h>       /* pow */

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#define M_PI	3.14159265358979323846  /* pi */
#define S0     0
#define S1     1
#define S2     2
#define S3     3
#define SX     4
#define SKIP_FRAMES 5

using namespace dlib;
using namespace std;

#define FACE_DOWNSAMPLE_RATIO 2
//#define SKIP_FRAMES 2

namespace cv
{
    using std::vector;
}

class TestClass
{
public:
//    FaceTracking faceTracker;

    TestClass();
    void draw_polyline(cv::Mat &img,
                       const dlib::full_object_detection& d,
                       const int start, const int end, bool isClosed = false);

    void render_face(cv::Mat &img, const dlib::full_object_detection& d);

    float Area(float semi_major_axis,
                          float semi_minor_axis);

    float RMSE( cv::vector<cv::Point> ground_truth,
                cv::vector<cv::Point> fitted_shapes);

    int state = S0;
    int next_state = S0;
    int counts = 0;

    cv::vector<cv::Point> ground_truth;


};
