#ifndef  __SENDER_RUNNER_HDR__
#define __SENDER_RUNNER_HDR__

#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <thread>

using namespace std;

class ClientPublisher{

public:
    ClientPublisher();
    ~ClientPublisher();

    bool open(sl::InputType, int sn, string prefix);
    void start();
    bool next();
    void close();
    void stop();

    int getSerialNumber() { return stoi(serial_number); }
    sl::Mat getImage_SL() { return zed_image; }
    cv::Mat getImage_CV() { return disp_image; }     
    sl::CameraParameters getCameraParameters() { 
        return zed.getCameraInformation().camera_configuration.calibration_parameters.left_cam; 
    }
    sl::Mat getPointCloud() { return point_cloud; }
    void setCameraPose_Eigen(const Eigen::Affine3f &a) { board_to_init_cam_pose = a; }
    void setCameraPose_SL(const sl::Transform &a) { cam_pose = a; }
    Eigen::MatrixXf getKeyPointData() { return kpt_data; }
    sl::Bodies getBodies() { return bodies; }
    sl::Objects getObjects() { return objects; }
    Eigen::Affine3f getCameraPose_Eigen() { return board_to_init_cam_pose; }
    sl::Transform getCameraPose_SL() { return cam_pose; }
    void updateCameraPose(Eigen::Affine3f cam_pose) { board_to_cam_pose = cam_pose; }

    

    // print
    bool isPrint;
    bool isExit;
    
private:
    void printPointCloud();
    // void transformBodies();
    // void transformObjects();


    sl::Camera zed;
    sl::InitParameters init_parameters;
    sl::BodyTrackingParameters body_tracking_parameters;
    sl::BodyTrackingRuntimeParameters body_runtime_parameters;
    sl::ObjectDetectionParameters object_detection_parameters;
    sl::ObjectDetectionRuntimeParameters object_detection_runtime_parameters;
    void work(sl::Camera& zed);
    std::thread runner;
    bool running;

    sl::Mat zed_image;
    std::string serial_number;
    cv::Mat disp_image;
    std::string wnd_name;

    // ADD
    sl::Bodies bodies;
    sl::Objects objects;
    
    sl::Mat point_cloud;
    Eigen::MatrixXf kpt_data;
    std::map<string, int> offset_parts;
    std::ofstream ofs;


    bool isCamRef;
    Eigen::Affine3f board_to_init_cam_ref;
    Eigen::Affine3f board_to_init_cam_pose;
    Eigen::Affine3f board_to_cam_pose;
    sl::Transform cam_pose;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

};

#endif // ! __SENDER_RUNNER_HDR__
