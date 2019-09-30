#include "test.h"

TestClass::TestClass(){

}

void TestClass::draw_polyline(cv::Mat &img, const dlib::full_object_detection& d,
                              const int start, const int end, bool isClosed)
{
    std::vector <cv::Point> points;
    for (int i = start; i <= end; ++i)
    {
        points.push_back(cv::Point(d.part(i).x(), d.part(i).y()));
    }
    cv::polylines(img, points, isClosed, cv::Scalar(255,0,0), 2, 16);

//    cv::imshow("test",img);

}

void TestClass::render_face (cv::Mat &img, const dlib::full_object_detection& d)
{
    DLIB_CASSERT
    (
     d.num_parts() == 68,
     "\n\t Invalid inputs were given to this function. "
     << "\n\t d.num_parts():  " << d.num_parts()
     );

    draw_polyline(img, d, 0, 16);           // Jaw line
    draw_polyline(img, d, 17, 21);          // Left eyebrow
    draw_polyline(img, d, 22, 26);          // Right eyebrow
    draw_polyline(img, d, 27, 30);          // Nose bridge
    draw_polyline(img, d, 30, 35, true);    // Lower nose
    draw_polyline(img, d, 36, 41, true);    // Left eye
    draw_polyline(img, d, 42, 47, true);    // Right Eye
    draw_polyline(img, d, 48, 59, true);    // Outer lip
    draw_polyline(img, d, 60, 67, true);    // Inner lip

}

float TestClass::Area(float semi_major_axis,
                      float semi_minor_axis){
    return M_PI*semi_major_axis*semi_minor_axis/4;
}

float TestClass::RMSE( cv::vector<cv::Point> ground_truth,
                       cv::vector<cv::Point> fitted_shapes){
    float result;
    for(int i=0; i<ground_truth.size(); i++){
        //cout << i << "\n";
        result += pow( ( fitted_shapes[i].x - ground_truth[i].x ), 2 ) + pow( ( fitted_shapes[i].y - ground_truth[i].y ), 2 );
        //cout << result << "\n";
    }
    result = sqrt (result) / ( sqrt ( pow( ( ground_truth[45].x - ground_truth[36].x ), 2 ) +
            pow( ( ground_truth[45].y - ground_truth[36].y ), 2 ) ) ) / ground_truth.size();
    //cout << result << "\n";
    return result;
}

int main(int argc, char** argv)
{
    TestClass testClass;

    cv::Mat im;
    cv::Mat im_small, im_display;

    cv::RNG rng(12345);
    try
    {

        // Load face detection and pose estimation models.
                std::cout << "Loading Model..." << "\n";
                frontal_face_detector detector = get_frontal_face_detector();
                shape_predictor pose_model;
                deserialize("/home/nipun/MPSYS/Q5/Humanoid_Robotics/Project/face_detection_ws/src/face_detection/shape_predictor_68_face_landmarks.dat") >> pose_model;
                std::cout << "Load Complete\n";
                cv::Mat temp;
                cv::VideoCapture cap(1);
                image_window win, eye;

                while (!win.is_closed())
                {

                    // Grab a frame
                    cap >> temp;

                    //Mat temp_gray;
                    //cvtColor(temp, temp_gray, COLOR_BGR2GRAY);  // Convert to gray scale
                    //equalizeHist(temp_gray, temp_gray); // Equalize histogram

                    //cv::resize(temp, temp, Size(640,480));
                    cv_image<dlib::bgr_pixel> cimg(temp);

                    // Detect faces
                    std::vector<dlib::rectangle> faces = detector(cimg);

                    // Find the pose of each face.
                    std::cout << faces.size() << " face detected!" << "\n"; // Show number of face detected
                    std::vector<full_object_detection> shapes;

                    std::vector<float> area_all;
                    std::vector<cv::RotatedRect> all_ellipse;


                    std::vector<float> RMSE_R;

                    // TABLE[state][x] -> next_state
                    const uint8_t TABLE[5][4] = {  // state tracking table
                    // |Current|   Next State   |
                    // | State | x = 0  1  2  3 |
                        /*S0*/     {S0,S1,S2,SX},
                        /*S1*/     {S0,S1,SX,S3},
                        /*S2*/     {S0,SX,S2,S3},
                        /*S3*/     {SX,S1,S2,S3},  // S3 is the initial state
                        /*SX*/     {S0,SX,SX,S3}   // SX is the error state
                    };

                    cv::vector< cv::vector<cv::Point> > all_land_mark;

                    if(faces.size()>0){

                        for ( int i = 0; i < faces.size(); i++ )
                        {
                            shapes.push_back(pose_model(cimg, faces[i]));
                        }

                        for ( int i = 0; i < faces.size(); i++ ){
                            cv::vector<cv::Point> land_mark;
                            for( int j = 0; j < 68; j++ )
                            {
                                cv::Point temp;
                                temp.x = shapes[i].part(j)(0);
                                temp.y = shapes[i].part(j)(1);
                                land_mark.push_back(temp);
                            }
                            all_land_mark.push_back(land_mark);
                        }

                        if( testClass.state == S0 && faces.size() == 1 ){ // Initial state, Initialize ground truth
                            testClass.ground_truth = all_land_mark[0];
                            std::cout << "State " << testClass.state << "\n";
                            testClass.state++;
                        }/*else{
                            state++;
                        }*/

                        if(testClass.state == S1 && faces.size() >= 1){ // Tracking state
                            for( int i=0; i<faces.size(); i++ ){
                                RMSE_R.push_back( testClass.RMSE( testClass.ground_truth, all_land_mark[i] ) );
                            }
                            cout << "State " << testClass.state << "\n";
                            testClass.state++;
                        }/*else{./T
                            state = SX;
                        }*/

                        if( testClass.state = S2 ){
                            for( int i = 0; i < RMSE_R.size(); i++ ){
                                cout << RMSE_R[i] << "\n";
                            }
                            std::vector<float>::iterator result;
                            result = std::min_element( RMSE_R.begin(), RMSE_R.end() );
                            int min_index = std::distance( RMSE_R.begin(), result );

                            cv::RotatedRect minEllipse;
                            minEllipse = cv::fitEllipse(all_land_mark[min_index]);
                            //all_ellipse.push_back(minEllipse);
                            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                            char str[200];
                            sprintf( str,"TRACKING");
                            putText( temp, str, minEllipse.center, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255), 3, 8, false );

                            cout << "State " << testClass.state << "\n";
                            testClass.state--;
                        }

                        if ( testClass.counts % SKIP_FRAMES == 0 && faces.size() == 1 ){  // Time to update ground truth
                            testClass.state--;
                            cout << "State " << testClass.state << "\n";
                        }

                    }

                    cv_image<dlib::bgr_pixel> debug(temp);
                    win.set_image(debug);
                }



//        cv::VideoCapture cap(1);
//        if (!cap.isOpened())
//        {
//            cerr << "Unable to connect to camera" << endl;
//            return 1;
//        }

//        image_window win;

//        // Load face detection and pose estimation models.
//        frontal_face_detector detector = get_frontal_face_detector();
//        shape_predictor pose_model;
//        deserialize("/home/nipun/MPSYS/Q5/Humanoid_Robotics/Project/face_detection_ws/src/face_detection/shape_predictor_5_face_landmarks.dat") >> pose_model;

//        int count = 0;
//        std::vector<rectangle> faces;
//        std::vector<full_object_detection> shapes;

//        for(;;){
//            // Grab a frame
//            cap >> im;

//            // Resize image for face detection
//            cv::resize(im, im_small, cv::Size(), 1.0/FACE_DOWNSAMPLE_RATIO, 1.0/FACE_DOWNSAMPLE_RATIO);

//            // Change to dlib's image format. No memory is copied.
//            cv_image<bgr_pixel> cimg_small(im_small);
//            cv_image<bgr_pixel> cimg(im);

//            if ( count % SKIP_FRAMES == 0 )
//            {
////                faces = detector(cimg_small);
//                faces = detector(cimg);
//            }

//            // Find the pose of each face.

//            if(faces.size() > 0){

//                std::cout << "Detected" << std::endl;

//                for (unsigned long i = 0; i < faces.size(); ++i)
//                {
//                    // Resize obtained rectangle for full resolution image.
////                     rectangle r(
////                                   (long)(faces[i].left() * FACE_DOWNSAMPLE_RATIO),
////                                   (long)(faces[i].top() * FACE_DOWNSAMPLE_RATIO),
////                                   (long)(faces[i].right() * FACE_DOWNSAMPLE_RATIO),
////                                   (long)(faces[i].bottom() * FACE_DOWNSAMPLE_RATIO)
////                                );

//                    // Landmark detection on full sized image
//                    full_object_detection shape = pose_model(cimg, faces[i]);
//                    shapes.push_back(shape);

//                    // Custom Face Render
//    //                testClass.render_face(im, shape);
//                }

//                cv::vector<cv::vector<cv::Point> > all_faces_land_mark;

//                for ( int i = 0; i < faces.size(); i++ ){
//                    cv::vector<cv::Point> land_mark;
//                    for( int j = 0; j < 16; j++ ){
//                        cv::Point temp;
//                        temp.x = shapes[i].part(j)(0);
//                        temp.y = shapes[i].part(j)(1);
//                        land_mark.push_back(temp);
//                    }
//                    all_faces_land_mark.push_back(land_mark);
//                }

//                for(int k = 0; k < all_faces_land_mark[0].size(); k++){
//                   std::cout <<  all_faces_land_mark[0][k] << std::endl;
//                }

//                cv::RotatedRect minEllipse_;
//                minEllipse_ = cv::fitEllipse(all_faces_land_mark[0]);
//                char str[200];

//                sprintf( str, "TRACKING");
//                putText( im, str, minEllipse_.center, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 3, 8, false );

////                cv::imshow("frame",im);
//            }
//            else {
//                char str[200];

//                sprintf( str, "ERROR");
//                putText( im, str, cv::Point(im.rows/2,im.cols/2), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 3, 8, false );
//            }


//            cv::imshow("frame",im);



//            if( cv::waitKey(10) == 27 ) break;
//        }





        // Grab and process frames until the main window is closed by the user.
//        while(!win.is_closed())
//        {

//            // Grab a frame
//            cv::Mat temp;
//            if (!cap.read(temp))
//            {
//                break;
//            }

//            cv_image<bgr_pixel> cimg(temp);

//            // Detect faces
//            std::vector<rectangle> faces = detector(cimg);
//            // Find the pose of each face.
//            std::vector<full_object_detection> shapes;
//            for (unsigned long i = 0; i < faces.size(); ++i)
//                shapes.push_back(pose_model(cimg, faces[i]));

//            // Display it all on the screen
//            win.clear_overlay();
//            win.set_image(cimg);
//            win.add_overlay(render_face_detections(shapes));
//        }
    }
    catch(serialization_error& e)
    {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
    }
    catch(exception& e)
    {
        cout << e.what() << endl;
    }
}

