#include "face_tracking.h"

FaceTracking::FaceTracking(){
    face_found = false;
}

void FaceTracking::initialize(std::__cxx11::string path_){
    detector = dlib::get_frontal_face_detector();

    std::cout << path_ << std::endl;    

    dlib::deserialize(path_) >> pose_model;
}

void FaceTracking::trackLandmark(cv::Mat &input_image_, cv::Mat &output_image_,std::string message){
      cv::Mat image_ = input_image_;
      dlib::cv_image<dlib::bgr_pixel> cimg(image_);
      std::vector<dlib::rectangle> cfaces;

//      std::cout << "testing..." << std::endl;
//      std::string message_;
//      message_ = "In the tracking loop";

      int tracking_index;
      std::vector<dlib::rectangle> faces = detector(cimg);

      char str[200];
      sprintf( str, "TRACKING 2");
      putText( image_, str, cv::Point(image_.rows/2,image_.cols/2), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 3, 8, false );

//      if( faces.size() > 0 ){

//          std::cout << "found faces" << std::endl;

//          face_found = true;
//          if( ground_truth.size() > 0 ) tracking_index = getTrackingIndex(cimg,faces);
//          cfaces = getCloseFaces(faces,tracking_index);

//          if(cfaces.size() > 0){

//              cv::vector< cv::vector<cv::Point> > all_land_mark;
//              cv::vector< cv::vector<cv::Point> > jaw_line;
//              int max_area_index;
//              message_ = "cfaces are higer than 0 : " + std::to_string(cfaces.size());
//              std::cout << "cfaces are higer than 0 " << std::endl;
//              getMaxAreaIndex(cimg,cfaces,all_land_mark,jaw_line,max_area_index);
//              startTracking(image_,cfaces,all_land_mark,jaw_line,max_area_index);
//          }

//          else
//          {
//              message_ = "Error with cfaces ";
//              std::cout << "Error with cfaces" << std::endl;
//          }
          
////          if( cfaces.size() > 0 ){
////              cv::vector< cv::vector<cv::Point> > all_land_mark;
////              cv::vector< cv::vector<cv::Point> > jaw_line;
////              int max_area_index;


////          }
//      }

//      else{
//              face_found = false;
//              message_ = "Error with faces ";
//              std::cout << "did not found faces" << std::endl;
//      }

//      cfacesize = cfaces.size();
      output_image_ = image_;
//      message = message_;
      std::cout << "===============" << std::endl;
}

int FaceTracking::getTrackingIndex(dlib::cv_image<dlib::bgr_pixel> img_, std::vector<dlib::rectangle> faces_){
    std::vector<dlib::full_object_detection> all_shapes;
    cv::vector< cv::vector<cv::Point> > all_faces_land_mark;
    std::vector<float> TEMP_RMSE_R;

    for( int i = 0; i < faces_.size(); i++ ){
        all_shapes.push_back( pose_model( img_, faces_[i] ) );
    }
    for ( int i = 0; i < faces_.size(); i++ ){
        cv::vector<cv::Point> land_mark;
        for( int j = 0; j < 68; j++ ){
            cv::Point temp;
            temp.x = all_shapes[i].part(j)(0);
            temp.y = all_shapes[i].part(j)(1);
            land_mark.push_back(temp);
        }
        all_faces_land_mark.push_back(land_mark);
    }
    for( int i = 0; i < faces_.size(); i++ ){
        TEMP_RMSE_R.push_back( RMSE( ground_truth, all_faces_land_mark[i] ) );
    }

    std::vector<float>::iterator result = std::min_element( TEMP_RMSE_R.begin(), TEMP_RMSE_R.end() );
    int tracking_index = std::distance( TEMP_RMSE_R.begin(), result );

    return tracking_index;
}

std::vector<dlib::rectangle> FaceTracking::getCloseFaces(std::vector<dlib::rectangle> faces_,int tracking_index){
    std::vector<dlib::rectangle> cfaces;
    for( int i = 0; i < faces_.size(); i++ )
    {
//        cfaces.push_back(faces_[i]);

        if(faces_[i].height() > RANGE_FOR_TRACKING && i == tracking_index ){
            cfaces.push_back(faces_[i]);
        }
        else if(faces_[i].height() > RANGE_FOR_DETECTED && i != tracking_index ){
            cfaces.push_back(faces_[i]);
        }
    }

    return cfaces;
}

float FaceTracking::calculateArea(float semi_major_axis, float semi_minor_axis){
    return M_PI*semi_major_axis*semi_minor_axis/4;
}

float FaceTracking::RMSE( std::vector<cv::Point> ground_truth, std::vector<cv::Point> fitted_shapes){
    float result;
    for(int i=0; i< ground_truth.size(); i++) {
        result += pow( ( fitted_shapes[i].x - ground_truth[i].x ), 2 ) + pow( ( fitted_shapes[i].y - ground_truth[i].y ), 2 );
    }
    result = sqrt (result /  ground_truth.size());
    return result;
}

void FaceTracking::getMaxAreaIndex(dlib::cv_image<dlib::bgr_pixel> &img_,
                                            std::vector<dlib::rectangle> &input,
                                            cv::vector<cv::vector<cv::Point> > &all_landmarks_,
                                            cv::vector< cv::vector<cv::Point> > &jaw_line,
                                            int &max_area_index)
{
    std::vector<dlib::full_object_detection> shapes;
    std::vector<cv::RotatedRect> minEllipse_all;
    std::vector<float> area_all;

    for( int i = 0; i < input.size(); i++ ) {
        shapes.push_back(pose_model(img_, input[i]));
    }
    for ( int i = 0; i < input.size(); i++ ){
        cv::vector<cv::Point> land_mark;
        cv::vector<cv::Point> jwl;
        for( int j = 0; j < 68; j++ ){
            cv::Point temp;
            temp.x = shapes[i].part(j)(0);
            temp.y = shapes[i].part(j)(1);
            land_mark.push_back(temp);
            if( j <= 16 ){
                jwl.push_back(temp);
            }
        }
        all_landmarks_.push_back(land_mark);
        jaw_line.push_back(jwl);
    }
    for ( int i = 0; i < input.size(); i++ ){
        cv::RotatedRect minEllipse;
        minEllipse = cv::fitEllipse(jaw_line[i]);

        minEllipse_all.push_back(minEllipse);
        float temp_area = calculateArea( minEllipse.size.width, minEllipse.size.height );
        area_all.push_back( temp_area );
    }

    //Find the closest index of interest faces
    std::vector<float>::iterator result = std::max_element( area_all.begin(), area_all.end() );
    max_area_index = std::distance( area_all.begin(), result );
}

void FaceTracking::startTracking(cv::Mat &image_,
                                          std::vector<dlib::rectangle> &cfaces,
                                          cv::vector<cv::vector<cv::Point> > &all_landmarks_,
                                          cv::vector<cv::vector<cv::Point> > &jaw_line,
                                          int &max_area_index){
    std::vector<float> RMSE_R;
    int min_index;

    if( state == Initialize ){
        // Initial state, Initialize ground truth
        ground_truth = all_landmarks_[max_area_index];
        state = Tracking;
    }

    if( state == Tracking){
        // Tracking state
        for( int i = 0; i < cfaces.size(); i++ ){
            RMSE_R.push_back( RMSE( ground_truth, all_landmarks_[i] ) );
        }

        std::vector<float>::iterator result = std::min_element( RMSE_R.begin(), RMSE_R.end() );
        min_index = std::distance( RMSE_R.begin(), result );



       cv::Rect cvface;
       cv::Point temp_predicted;
       temp_predicted.x = 24;
       temp_predicted.y = 44;

       int faceSize = cfaces[max_area_index].width();
       if (cfaces[max_area_index].width() < cfaces[max_area_index].height()){
           faceSize = cfaces[max_area_index].height();
       }

       int faceNewLeft = cfaces[max_area_index].left();
       int faceNewTop = cfaces[max_area_index].top();

       if (faceNewLeft < 0 ) faceNewLeft = faceNewLeft - faceNewLeft;
       if (faceNewTop < 0) faceNewTop = faceNewTop - faceNewTop;

       int endPointA = faceNewLeft + faceSize;
       int endPointB = faceNewTop + faceSize;

       if(endPointA <= image_.cols && endPointB <= image_.rows){
           cv::Rect rect(faceNewLeft, faceNewTop, faceSize, faceSize);
           cv::rectangle(image_, rect, cv::Scalar(255, 0, 0),2,8,0);
           cvface.x = faceNewLeft;
           cvface.y = faceNewTop;
           cvface.width = faceSize;
           cvface.height = faceSize;
        }

       else{
           int errorLeft,errorTop;
           if(endPointA >= image_.cols) errorLeft = endPointA - image_.cols;
           if(endPointB >= image_.rows) errorTop = endPointB - image_.rows;
           cv::Rect rect_((faceNewLeft - errorLeft), (faceNewTop - errorTop), faceSize, faceSize);
           cv::rectangle(image_, rect_, cv::Scalar(0, 0, 255),2,8,0);
       }

        std::string tempStr = std::to_string(min_index);

        cv::RotatedRect minEllipse_;
        minEllipse_ = cv::fitEllipse(jaw_line[min_index]);
        char str[200];

        char temp = min_index;
        sprintf( str, "TRACKING");
        putText( image_, str, minEllipse_.center, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 3, 8, false );


        if( minEllipse_.center.x >= 0 &&
            minEllipse_.center.x <= 640 &&
            minEllipse_.center.y >= 0 &&
            minEllipse_.center.x <= 480 ) minEllipse = minEllipse_;

        state = Updating;
    }

    if ( state == Updating ){
        ground_truth = all_landmarks_[min_index];
        state = Tracking;
    }
}



bool FaceTracking::requestDetectedRealTime(){
    return face_found;
}

int FaceTracking::requestFacesSize(){
    return cfacesize;
}

cv::RotatedRect FaceTracking::requestEllipseCenter(){
    return minEllipse;
}
