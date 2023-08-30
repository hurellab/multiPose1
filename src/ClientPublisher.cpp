#include "ClientPublisher.hpp"

static bool checkZEDState(const sl::ERROR_CODE& state) {
    if (state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error: " << state << std::endl;
        return false;
    }
    return true;
}


ClientPublisher::ClientPublisher() : running(false), isPrint(false), isExit(false), isCamRef(false)
{
    // init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    // init_parameters.camera_resolution = sl::RESOLUTION::HD2K;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_maximum_distance = 10.0f;
    init_parameters.camera_fps = 15;
}

ClientPublisher::~ClientPublisher()
{
    zed.close();
    ofs.close();
}

bool ClientPublisher::open(sl::InputType input, int sn, string prefix) {
    // already running
    if (runner.joinable())
        return false;

    init_parameters.input = input;

    serial_number = std::to_string(sn);
    if (!prefix.empty()) {
        init_parameters.input.setFromSVOFile((prefix + serial_number + ".svo").c_str());
    }
    
    auto state = zed.open(init_parameters);
    if(!checkZEDState(state)) return false;

    // in most cases in body tracking setup, the cameras are static
    sl::PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_as_static = false;
    state = zed.enablePositionalTracking(positional_tracking_parameters);
    if(!checkZEDState(state)) return false;

    // define the body tracking parameters, as the fusion can does the tracking and fitting you don't need to enable them here, unless you need it for your app
    body_tracking_parameters.body_format = sl::BODY_FORMAT::BODY_34;
    body_tracking_parameters.enable_tracking = true;
    body_tracking_parameters.enable_body_fitting = true;
    body_tracking_parameters.enable_segmentation = false; // designed to give person pixel mask
    body_tracking_parameters.detection_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
    body_tracking_parameters.instance_module_id = 0; // select instance ID
    body_runtime_parameters.detection_confidence_threshold = 60;
    state = zed.enableBodyTracking(body_tracking_parameters);
    if(!checkZEDState(state)) return false;
    
    // serial_number = std::to_string(zed.getCameraInformation().serial_number);

    sl::CalibrationParameters calibration_parameters 
        = zed.getCameraInformation().camera_configuration.calibration_parameters_raw;
    double fx = calibration_parameters.left_cam.fx;
    double fy = calibration_parameters.left_cam.fy;
    double cx = calibration_parameters.left_cam.cx;
    double cy = calibration_parameters.left_cam.cy;
    double k1 = calibration_parameters.left_cam.disto[0];
    double k2 = calibration_parameters.left_cam.disto[1];
    double p1 = calibration_parameters.left_cam.disto[2];
    double p2 = calibration_parameters.left_cam.disto[3];
    double k3 = calibration_parameters.left_cam.disto[4];
    double k4 = calibration_parameters.left_cam.disto[5];
    double k5 = calibration_parameters.left_cam.disto[6];
    double k6 = calibration_parameters.left_cam.disto[7];
    double s1 = calibration_parameters.left_cam.disto[8];
    double s2 = calibration_parameters.left_cam.disto[9];
    double s3 = calibration_parameters.left_cam.disto[10];
    double s4 = calibration_parameters.left_cam.disto[11];
    cameraMatrix = (cv::Mat_<double>(3,3)  << fx,0,cx, 0,fy,cy, 0,0,1);
    distCoeffs   = (cv::Mat_<double>(1,12) << k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4);

    object_detection_parameters.enable_tracking = true;
    object_detection_parameters.enable_segmentation = false; // designed to give person pixel mask
    object_detection_parameters.detection_model = sl::OBJECT_DETECTION_MODEL::PERSON_HEAD_BOX_ACCURATE;
    object_detection_parameters.instance_module_id = 1; // select instance ID
    object_detection_runtime_parameters.detection_confidence_threshold = 20;
    object_detection_runtime_parameters.object_class_filter = { sl::OBJECT_CLASS::PERSON };
    state = zed.enableObjectDetection(object_detection_parameters);
    if(!checkZEDState(state)) return false;
    return true;
}

void ClientPublisher::start()
{
    if (zed.isOpened()) {
        board_to_cam_pose = board_to_init_cam_pose;
        running = true;
        runner = std::thread(&ClientPublisher::work, this, std::ref(zed));
        wnd_name = "ZED S/N: " + serial_number;
        cv::namedWindow(wnd_name);
    }
}

void ClientPublisher::stop()
{
    running = false;
    if (runner.joinable())
        runner.join();

    zed.close();
}

void ClientPublisher::close()
{
    zed.close();
    zed_image.free();
    point_cloud.free();
}

void ClientPublisher::work(sl::Camera& zed)
{
    sl::Pose pose;

    while (running)
    {        
        if (zed.grab() != sl::ERROR_CODE::SUCCESS)
            continue;
            
        // display
        zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
        disp_image = cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::resize(disp_image, disp_image, cv::Size(int(disp_image.cols * 0.4), int(disp_image.rows * 0.4)));
        cv::imshow(wnd_name, disp_image);
    
        zed.retrieveObjects(objects, object_detection_runtime_parameters, object_detection_parameters.instance_module_id);
        zed.retrieveBodies(bodies, body_runtime_parameters, body_tracking_parameters.instance_module_id);
        zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::GPU, sl::Resolution(1280, 720));

        // transformBodies();
        // transformObjects();

        if (isPrint) {
            printPointCloud();
            isPrint = false;
        }
        // if (isExit) {
        //     break;
        // }
    }
    zed_image.free();
    point_cloud.free();
}

bool ClientPublisher::next()
{
    sl::Pose pose;

        if (zed.grab() != sl::ERROR_CODE::SUCCESS)
            return false;
            
        // display
        zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
        disp_image = cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::resize(disp_image, disp_image, cv::Size(int(disp_image.cols * 0.4), int(disp_image.rows * 0.4)));
        cv::imshow(wnd_name, disp_image);
    
        zed.retrieveObjects(objects, object_detection_runtime_parameters, object_detection_parameters.instance_module_id);
        zed.retrieveBodies(bodies, body_runtime_parameters, body_tracking_parameters.instance_module_id);
        zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::GPU, sl::Resolution(1280, 720));

        return true;

        // transformBodies();
        // transformObjects();

        // if (isPrint) {
        //     printPointCloud();
        //     isPrint = false;
        // }
        // if (isExit) {
        //     break;
        // }
    // }
}


void ClientPublisher::printPointCloud()
{
    cout << serial_number << "'s point cloud data is generating..." << endl;
    sl::Mat point_cloud_save;
    zed.retrieveMeasure(point_cloud_save, sl::MEASURE::XYZRGBA, sl::MEM::CPU, sl::Resolution(1280, 720));
    sl::float4 *point_cloud_ptr = point_cloud_save.getPtr<sl::float4>(sl::MEM::CPU);
    int width = point_cloud_save.getWidth();
    int height = point_cloud_save.getHeight();
    for (size_t y=0;y<height;y++) {
        for (size_t x=0;x<width; x++) {
            sl::float4 &point = point_cloud_ptr[y * width + x];
            if (!isValidMeasure(point.z)) // if the point is not valid, skip it
                continue;
            Eigen::Vector3f tf_point = board_to_cam_pose * Eigen::Vector3f(point.x, point.y, point.z);
            point.x = tf_point.x();
            point.y = tf_point.y();
            point.z = tf_point.z();
        }
    }
    auto write_suceed = point_cloud_save.write((serial_number + "_tf.ply").c_str(), sl::MEM::CPU);
    if(write_suceed == sl::ERROR_CODE::SUCCESS) 
        std::cout << "Current .ply file saving succeed" << std::endl;                
    else
        std::cout << "Current .ply file saving failed" << std::endl;
    point_cloud_save.free();

    // cout << serial_number << "'s body data is generating..." << endl;
    // ofstream ofs(serial_number + "_bodyData.obj");
    // for (int i=0; i<kpt_data.rows(); i++) {
    //     ofs << "v " << kpt_data.row(i) << endl;
    // } ofs.close();
    cout << serial_number << "'s print end" << endl;
}

// void ClientPublisher::transformBodies()
// {
//     for (auto &body: bodies.body_list) {
//         int kpt_count(0);
//         for (auto &kpt: body.keypoint) {
//             Eigen::Vector3f kpt_tf = board_to_cam_pose * Eigen::Vector3f(kpt.x, kpt.y, kpt.z);
//             kpt.x = kpt_tf.x();
//             kpt.y = kpt_tf.y();
//             kpt.z = kpt_tf.z();
//             kpt_data.row(kpt_count++) = kpt_tf;
//         }
//         for (auto &pt: body.bounding_box) {
//             Eigen::Vector3f pt_tf = board_to_cam_pose * Eigen::Vector3f(pt.x, pt.y, pt.z);
//             pt.x = pt_tf.x();
//             pt.y = pt_tf.y();
//             pt.z = pt_tf.z();
//         }
//     }
// }
// void ClientPublisher::transformObjects()
// {
//     for (auto &head: objects.object_list) {
//         for (auto &pt: head.head_bounding_box) {
//             Eigen::Vector3f pt_tf = board_to_cam_pose * Eigen::Vector3f(pt.x, pt.y, pt.z);
//             pt.x = pt_tf.x();
//             pt.y = pt_tf.y();
//             pt.z = pt_tf.z();
//         }
//     }
// }