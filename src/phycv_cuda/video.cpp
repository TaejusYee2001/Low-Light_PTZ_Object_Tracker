#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <iomanip>
#include <string>
#include <experimental/filesystem>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "vevid.cuh"
#include "video.hpp"
#include "options.hpp"
#include "detect_net.hpp"
#include "controls.hpp"

using namespace std; 
using namespace cv; 

namespace fs = experimental::filesystem; 

// TODO: add option to specify CSI or USB camera in constructor. 
WebCam::WebCam(int device, int width, int height)
    : m_device(device), m_width(width), m_height(height), m_frame_count(0), m_exit(false), m_new_frame(false) 
{
    const std::string gst_str = "nvarguscamerasrc ! "
                                "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! "
                                "nvvidconv flip-method=2 ! "
                                "video/x-raw, format=(string)BGRx ! "
                                "videoconvert ! "
                                "video/x-raw, format=(string)BGR ! "
                                "appsink";
    //m_webcam.open(gst_str, CAP_GSTREAMER);
    m_webcam.open(0, CAP_V4L2);
    m_webcam.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    m_webcam.set(CAP_PROP_FRAME_WIDTH, m_width);
    m_webcam.set(CAP_PROP_FRAME_HEIGHT, m_height);
    m_webcam.set(CAP_PROP_FPS, 30);


}

WebCam::~WebCam() {
    m_webcam.release();
}

void WebCam::start_capturing() {
    while (!m_exit) {
        Mat frame;
        m_webcam >> frame;
        
        if (frame.cols != m_width || frame.rows != m_height) {
            resize(frame, frame, cv::Size(m_width, m_height));
        }

        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_frame = frame;
            m_new_frame = true;
        }

        m_frame_cv.notify_one();
        m_frame_count++;
    }
}

void WebCam::stop_capturing() {
    m_exit = true;
}

Mat WebCam::get_frame() {
    unique_lock<mutex> lock(m_mutex);
    m_frame_cv.wait(lock, [this]() { return m_new_frame; });
    m_new_frame = false;
    return m_frame;
}

int WebCam::get_frame_count() const {
    return m_frame_count;
}

int WebCam::get_width() const {
    return m_width; 
}

int WebCam::get_height() const {
    return m_height; 
}

Window::Window(const string& window1_name, const string& window2_name)
    : m_window1_name(window1_name), m_window2_name(window2_name), m_exit(false), m_frame_count(0) 
{}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void Window::process_camera(WebCam& webcam, Flags* flags, Params* params) {

    Controller controller; 
    int motor_step = 2; 
    int focus_step = 100;
    int zoom_step = 100; 
    
    cout << "Using camera resolution:" << endl;
    cout << "   width = " << params->width << endl;
    cout << "   height = " << params->height << endl;

    Vevid vevid(webcam.get_width(), webcam.get_height(), params->S, params->T, params->b, params->G);
    //namedWindow(m_window1_name, WINDOW_NORMAL); 
    namedWindow(m_window2_name, WINDOW_NORMAL); 

    VideoWriter output; 

    if (flags->w_value != nullptr) {
        cout << "Writing video to " << flags->w_value << endl; 
        string output_path(flags->w_value); 

        size_t last_slash = output_path.find_last_of('/'); 
        if (last_slash == string::npos) {
            cout << "Error: Invalid output path format: " << flags->w_value << endl; 
            exit(1); 
        }

        string out_dir = output_path.substr(0, last_slash); 

        if (!fs::exists(out_dir)) {
            if (!fs::create_directories(out_dir)) {
                cout << "Error: Could not create directory " << out_dir << endl; 
                exit(1); 
            }
        }
        int fourcc = VideoWriter::fourcc('a', 'v', 'c', '1'); 
        int fps = 10; 
        //Size frame_size(params->width * 2, params->height); 
        Size frame_size(params->width, params->height); 

        output.open(output_path, fourcc, fps, frame_size); 

        if (!output.isOpened()) {
            cout << "Error: Could not write video to " << flags->w_value << endl; 
            exit(1); 
        }
    }

    uchar3* d_image; 
    DetectNet net(d_image, webcam.get_width(), webcam.get_height()); 
    if (flags->d_flag) {
        net.create(); 
    }

    // Initialize Kalman filter parameters
    const int stateSize = 4;  // [x, y, vx, vy]
    const int measSize = 2;    // [x, y]
    const int contrSize = 0;
    int type = CV_32F;

    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
    cv::setIdentity(kf.transitionMatrix);
    cv::Mat H = cv::Mat::zeros(measSize, stateSize, type);
    H.at<float>(0) = 1.0f;
    H.at<float>(5) = 1.0f;
    kf.measurementMatrix = H;
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-6));
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    const float pixels_per_degree = 10.0f;

    while (!m_exit) {
        Mat frame = webcam.get_frame();
        //Mat original = frame.clone(); 

        /*
        if (flags->d_flag) {
            net.run(original); 
        }
        */
        

        //imshow(m_window1_name, original);
        imshow(m_window1_name, frame);
        vevid.run(frame, flags->t_flag, flags->l_flag);

        if (flags->d_flag) {
            net.run(frame); 
            
            // Calculate the center of the bounding box
            float box_center_x = (net.m_box_left + net.m_box_right) / 2.0;
            float box_center_y = (net.m_box_top + net.m_box_bottom) / 2.0;

            // Perform Kalman filter prediction
            cv::Mat prediction = kf.predict();

            // Extract predicted x and y values from the state vector
            float predicted_x = prediction.at<float>(0);
            float predicted_y = prediction.at<float>(1);

            // Print predicted x and y values
            cout << "Predicted X: " << predicted_x << ", Predicted Y: " << predicted_y << endl;

            // Convert predicted pixel coordinates to motor positions
            uint16_t new_position_x = static_cast<uint16_t>(predicted_x / pixels_per_degree);
            uint16_t new_position_y = static_cast<uint16_t>(predicted_y / pixels_per_degree);
            
            if (net.m_num_detections == 1) {
                if (m_frame_count % 2 == 0) {
                    if (predicted_x > (webcam.get_width() / 2) + (webcam.get_width() / 8)) {
                        controller.set(OPT_MOTOR_X, controller.get(OPT_MOTOR_X) - motor_step); 
                    }
                    else if (predicted_x < (webcam.get_width() / 2) - (webcam.get_width() / 8)){
                        controller.set(OPT_MOTOR_X, controller.get(OPT_MOTOR_X) + motor_step); 
                    }
                }
                
                else {
                    if (predicted_y > (webcam.get_height() / 2) + (webcam.get_height() / 8)) {
                        controller.set(OPT_MOTOR_Y, controller.get(OPT_MOTOR_Y) + motor_step);
                    }
                    else if (predicted_y < (webcam.get_height() / 2) - (webcam.get_height() / 8)) {
                        controller.set(OPT_MOTOR_Y, controller.get(OPT_MOTOR_Y) - motor_step); 
                    }
                }
                
            }
            
            // Update Kalman filter with observed measurement (bounding box center)
            cv::Mat measured_state = (cv::Mat_<float>(2, 1) << box_center_x, box_center_y);
            kf.correct(measured_state);
            
        }
        

        display_fps(frame); 
        
        imshow(m_window2_name, frame); 
        m_frame_count++;

        /*
        if (flags->w_value != nullptr) {
            
            Mat concat_frame; 
            hconcat(original, frame, concat_frame); 
            output.write(concat_frame); 
            
            output.write(frame); 
        }
        */

        char key = waitKey(1);
        if (key == 27) {
            m_exit = true;
        }
    }
}

void Window::display_fps(Mat& frame) {
    static chrono::steady_clock::time_point last_time = chrono::steady_clock::now();
    chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
    chrono::duration<double> duration = current_time - last_time;
    double fps = 1.0 / duration.count();

    ostringstream ss;
    ss << "FPS: " << fixed << setprecision(1) << fps;

    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 2;
    int thickness = 4;
    int baseline = 0;
    Size text_size = getTextSize(ss.str(), font_face, font_scale, thickness, &baseline);

    Point text_org(frame.cols - text_size.width - 10, frame.rows - text_size.height - 10);
    putText(frame, ss.str(), text_org, font_face, font_scale, Scalar(255, 255, 255), thickness);

    last_time = current_time;
}

void Window::process_image(Mat& frame, Flags* flags, Params* params) {
    cout << "Running VEViD on input image " << flags->i_value << endl; 

    frame = imread(flags->i_value); 

    if (flags->r_value == nullptr) {
        params->width = frame.cols; 
        params->height = frame.rows; 
        cout << "Custom resolution parameters not specified, using original image resolution:" << endl;
        cout << "   width = " << params->width << endl;
        cout << "   height = " << params->height << endl;
    }

    Vevid vevid(params->width, params->height, params->S, params->T, params->b, params->G); 

    if (frame.empty()) {
        cout << "Error: Could not load the input image " << endl; 
        exit(1); 
    }

    resize(frame, frame, Size(params->width, params->height)); 
    if (flags->w_value == nullptr) {
        namedWindow("Original Image", WINDOW_NORMAL); 
        namedWindow("VEViD-Enhanced Image", WINDOW_NORMAL); 
        imshow("Original Image", frame); 
    }
    vevid.run(frame, flags->t_flag, flags->l_flag); 

    if (flags->d_flag) {
        uchar3* d_image; 
        DetectNet net(d_image, params->width, params->height); 
        net.create(); 
        net.run(frame); 
    }

    if (flags->w_value == nullptr) {
        imshow("VEViD-Enhanced Image", frame); 
    }

    waitKey();

    if (flags->w_value != nullptr) {
        cout << "Writing image to " << flags->w_value << endl;

        string output_path(flags->w_value); 

        size_t last_slash = output_path.find_last_of('/');
        if (last_slash == string::npos) {
            cout << "Error: Invalid output path format: " << flags->w_value << endl; 
            exit(1); 
        }

        string out_dir = output_path.substr(0, last_slash); 

        if (!fs::exists(out_dir)) {
            if (!fs::create_directories(out_dir)) {
                cout << "Error: Could not create directory " << out_dir << endl;
                exit(1);
            }
        }

        if (!imwrite(flags->w_value, frame)) {
            cout << "Error: Could not write the output image to " << flags->w_value << endl; 
            exit(1); 
        } 
    }
}

void Window::process_video(VideoCapture& camera, Mat& frame, Flags* flags, Params* params) {
    cout << "Running VEViD on input video " << flags->v_value << endl;

    camera.open(flags->v_value);
    if (!camera.isOpened()) {
        cout << "Error: Could not open video file" << flags->v_value << endl;
        exit(1);
    }

    bool change_dims = false;
    if (flags->r_value != nullptr) {
        change_dims = true; 
    }
    else {
        params->width = camera.get(CAP_PROP_FRAME_WIDTH); 
        params->height = camera.get(CAP_PROP_FRAME_HEIGHT); 
        cout << "Custom resolution parameters not specified, using original video resolution:" << endl;
        cout << "   width = " << params->width << endl;
        cout << "   height = " << params->height << endl;
    }

    if (flags->w_value == nullptr) {
        namedWindow("Original Video", WINDOW_NORMAL);
        namedWindow("VEViD-Enhanced Video", WINDOW_NORMAL);
    }
    Vevid vevid(params->width, params->height, params->S, params->T, params->b, params->G);

    VideoWriter output;

    if (flags->w_value != nullptr) {
        cout << "Writing video to " << flags->w_value << endl;
        string output_path(flags->w_value);

        size_t last_slash = output_path.find_last_of('/');
        if (last_slash == string::npos) {
            cout << "Error: Invalid output path format: " << flags->w_value << endl; 
            exit(1); 
        }

        string out_dir = output_path.substr(0, last_slash); 

        if (!fs::exists(out_dir)) {
            if (!fs::create_directories(out_dir)) {
                cout << "Error: Could not create directory " << out_dir << endl;
                exit(1);
            }
        }
        int fourcc = VideoWriter::fourcc('a', 'v', 'c', '1');
        int fps = 30;
        Size frame_size(params->width, params->height);

        output.open(output_path, fourcc, fps, frame_size);

        if (!output.isOpened()) {
            cout << "Error: Could not write video to " << flags->w_value << endl;
            exit(1); 
        }
    }

    uchar3* d_image;
    DetectNet net(d_image, params->width, params->height);
    if (flags->d_flag) {
        net.create();
    }

    while (true) {
        camera >> frame;

        if (frame.empty()) {
            break;
        }

        if (change_dims) {
            resize(frame, frame, Size(params->width, params->height));
        }

        if (flags->w_value == nullptr) {
            imshow("Original Video", frame);
        }
    
        vevid.run(frame, flags->t_flag, flags->l_flag);

        if (flags->d_flag) {
            net.run(frame);
        }

        if (flags->w_value == nullptr) {
            imshow("VEViD-Enhanced Video", frame);
        }
        
        if (flags->w_value != nullptr) {
            output.write(frame); 
        }

        char key = waitKey(1);
        if (key == 27) {
            break;
        }
    }
}