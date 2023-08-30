///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "ClientPublisher.hpp"
#include "GLViewer.hpp"

#include <iostream>
#include <mutex>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;

void convertAffineEigen2SL(const Eigen::Affine3f &inp, sl::Transform &out);
void setFusedBodies(const map<int, sl::Bodies>& camera_bodies, sl::Bodies& fused_bodies);
void transformObjects(const Eigen::Affine3f& cam_pose, sl::Objects& camera_objects);
void transformBodies(const Eigen::Affine3f& cam_pose, sl::Bodies& camera_bodies);

int main(int argc, char **argv) 
{
    string svo_prefix;
    if (argc != 4)
    {
        cerr << "./ZED_multiMocap [json file] [prefix] [top view]" << endl;
        return 1;
    }
    svo_prefix = string(argv[2]);
    int topView = atoi(argv[3]);

    // Read json file containing the configuration of your multicamera setup.
    std::string json_config(argv[1]);
    std::vector<sl::FusionConfiguration> configurations = sl::readFusionConfigurationFile(json_config, sl::COORDINATE_SYSTEM::IMAGE, sl::UNIT::METER);
    
    if (configurations.empty()) {
        std::cout << "Empty configuration File." << std::endl;
        return EXIT_FAILURE;
    }

    // Check if the ZED camera should run within the same process or if they are running on the edge.
    std::vector<sl::CameraParameters> camera_parameters;
    // Main Loop
    map<int, map<int, sl::Objects>> camera_objects;
    map<int, map<int, sl::Bodies>> camera_bodies;
    map<int, sl::Mat> camera_pointClouds;
    map<int, sl::Transform> camera_poses;
    sl::Mat point_cloud;
    Eigen::Affine3f camPose;
    sl::Transform camPose_;
    for (sl::FusionConfiguration& conf: configurations) {
        ClientPublisher zed;
        // if the ZED camera should run locally, then start a thread to handle it
        if(conf.communication_parameters.getType() == sl::CommunicationParameters::COMM_TYPE::INTRA_PROCESS)
        {   
            std::cout << "Try to open ZED " <<conf.serial_number << ".." << std::flush;
            auto state = zed.open(conf.input_type, conf.serial_number, svo_prefix);
            if (state) {
                std::cout << ". ready !" << std::endl;
                sl::float3 t_sl = conf.pose.getTranslation();
                sl::float4 q_sl = conf.pose.getOrientation();
                Eigen::Translation3f t(t_sl.x, t_sl.y, t_sl.z);
                Eigen::Quaternionf q(q_sl.w, q_sl.x, q_sl.y, q_sl.z); q.normalize();
                Eigen::Affine3f a = t * q;
               
                camera_parameters.push_back(zed.getCameraParameters());
                zed.setCameraPose_Eigen(a);
                zed.setCameraPose_SL(conf.pose);
                // camPose = monitor_pose * zed.getCameraPose_Eigen();
                camPose = zed.getCameraPose_Eigen();
                convertAffineEigen2SL(camPose, camPose_);
                camera_poses[conf.serial_number] = camPose_;
                bool gl_viewer_available = true;
                bool quit = false;
                int frameNo(-1);
                while(gl_viewer_available && !quit)
                {
                    if(!zed.next()) break;
                    frameNo++;
                    if(conf.serial_number==topView)
                    {
                        auto obj = zed.getObjects();
                        if(!obj.is_tracked) continue;
                        transformObjects(camPose, obj);
                        obj.object_list.erase(std::remove_if(obj.object_list.begin(), obj.object_list.end(),
                                        [](sl::ObjectData iter){return iter.head_position.z<1.;}), obj.object_list.end()); //doldoli
                        camera_objects[frameNo][conf.serial_number] = obj;
                    }
                    else
                    {
                        auto body = zed.getBodies();
                        if(!body.is_tracked) continue;
                        transformBodies(camPose, body);
                        camera_bodies[frameNo][conf.serial_number] = body;
                    }
                    cout<<"\rcollected "<<frameNo<<" frames"<<flush;
                }
                zed.close();
                cout<<endl;
            }
        }
    }

    GLViewer viewer;
    viewer.init(argc, argv, camera_parameters, configurations);
    // viewer.disableVisRaw();
    char key = ' ';
    int frame_count = 0;
    int noFrames = max(camera_objects.size(), camera_bodies.size());
    bool gl_viewer_available = true;
    bool quit = false;
    bool pause = false;

    ofstream ofs("posture.dat");
    ofstream ofs_BV("posture_BV.dat");
    while( gl_viewer_available && !quit)
    {
        // k4a_device_get_capture(device, &capture, 1000);
        // color_image = k4a_capture_get_color_image(capture);
        // cv::Mat image = color_to_opencv(color_image);
        // monitorTracker.Process_Current_Frame(image);
        // Eigen::Affine3f monitor_pose = monitorTracker.Get_Monitor_Pose();
        // monitorTracker.Render();
        // k4a_capture_release(capture);
        // k4a_image_release(color_image);
        
        // setFusedBodies(camera_bodies, fused_bodies);
        sl::Bodies fusedBody, fusedBody_BV;
        fusedBody.body_format = sl::BODY_FORMAT::BODY_34;
        fusedBody_BV.body_format = sl::BODY_FORMAT::BODY_34;
        ofs<<"f "<<frame_count<<endl; 
        ofs_BV<<"f "<<frame_count<<endl; 
        if(camera_objects[frame_count][topView].is_tracked)
        {
            int n = camera_objects[frame_count][topView].object_list.size();
            Eigen::MatrixXf refHeads(n, 2);
            Eigen::VectorXi refIDs(n);
            n=0;
            for (auto &obj : camera_objects[frame_count][topView].object_list) {
                if (obj.tracking_state != sl::OBJECT_TRACKING_STATE::OK) continue;
                refHeads.row(n) = Eigen::RowVector2f(obj.head_position.x, obj.head_position.y);
                refIDs(n++) = obj.id;
            }
            refHeads.conservativeResize(n+1, 2);
            refIDs.conservativeResize(n+1);
            //generate matrices for each camera
            map<int,Eigen::MatrixXf> heads;
            for(auto post : camera_bodies[frame_count])
            {
                if(post.second.body_list.size()==0) continue;
                Eigen::MatrixXf dat(post.second.body_list.size(), 2);
                for(int i=0;i<dat.rows();i++){
                    auto head = post.second.body_list[i].keypoint[5];
                    dat.row(i) = Eigen::RowVector2f(head.x, head.y);
                }
                heads[post.first] = dat;
            }
            // map<int, double> dist;
            // map<int, Eigen::RowVector2f> diff;
            for (auto &obj : camera_objects[frame_count][topView].object_list) {
                if (obj.tracking_state != sl::OBJECT_TRACKING_STATE::OK) continue;
                Eigen::RowVector2f center(obj.head_position.x, obj.head_position.y);
                Eigen::MatrixXf fusedPosture, fusedBV;
                Eigen::VectorXf fusedConf, fusedConf_BV;              

                for(auto dat:heads)
                {
                    Eigen::Index idx, idx0;
                    if((dat.second.rowwise()-center).rowwise().squaredNorm().minCoeff(&idx)>0.25) continue;
                    (refHeads.rowwise()-dat.second.row(idx)).rowwise().squaredNorm().minCoeff(&idx0);
                    if(refIDs(idx0)!=obj.id) continue;
                    sl::BodyData* body = &camera_bodies[frame_count][dat.first].body_list[idx];
                    if(fusedPosture.rows()==0)
                    {
                        fusedPosture = Eigen::MatrixXf::Zero(34, 3);
                        fusedConf = Eigen::VectorXf::Zero(34);
                        fusedBV = Eigen::MatrixXf::Zero(sl::BODY_34_BONES.size(), 4);
                        fusedConf_BV = Eigen::VectorXf::Zero(sl::BODY_34_BONES.size());
                    }
                    // avg. keypoint
                    for(int i=0;i<34;i++)
                    {
                        auto p = body->keypoint[i];
                        float conf = body->keypoint_confidence[i];
                        fusedPosture.row(i) += Eigen::RowVector3f(p.x, p.y, p.z) * conf;
                        fusedConf(i) += conf;
                    }
                    // avg. BV
                    for(size_t i=0;i<sl::BODY_34_BONES.size();i++)
                    {
                        auto b0 = body->keypoint[(int)sl::BODY_34_BONES[i].first];
                        auto bv = body->keypoint[(int)sl::BODY_34_BONES[i].second] - b0;
                        float norm = bv.norm();
                        bv = bv/norm;
                        double conf = (body->keypoint_confidence[(int)sl::BODY_34_BONES[i].first]+body->keypoint_confidence[(int)sl::BODY_34_BONES[i].second])*0.5;
                        fusedBV.row(i) += Eigen::RowVector4f(bv.x, bv.y, bv.z, norm)*conf;
                        fusedConf_BV(i) += conf; 
                    }
                }
                obj.id ++;
                if(fusedPosture.rows()>0)
                {
                    //avg. kpt
                    fusedBody.is_tracked = true;
                    sl::BodyData body;
                    body.id = obj.id;
                    body.tracking_state = sl::OBJECT_TRACKING_STATE::OK;
                    std::vector<sl::float3> keypoints;
                    ofs<<"p "<<body.id<<" "<<center(0)*100<<" "<<center(1)*100<<endl;
                    ofs_BV<<"p "<<body.id<<" "<<center(0)*100<<" "<<center(1)*100<<endl;
                    for(int i=0;i<34;i++)
                    {
                        Eigen::RowVector3f p = fusedPosture.row(i)/fusedConf(i);
                        keypoints.push_back(sl::float3(p(0), p(1), p(2)));
                        p *= 100;
                        ofs<< p <<endl;
                    }
                    body.keypoint = keypoints;
                    fusedBody.body_list.push_back(body);

                    //avg. bv
                    fusedBody_BV.is_tracked = true;
                    Eigen::MatrixXf kpt = Eigen::MatrixXf::Zero(34, 3);
                    kpt.row(0)=Eigen::RowVector3f(keypoints[0].x, keypoints[0].y, keypoints[0].z);
                    for(int i=0; i<sl::BODY_34_BONES.size();i++)
                    {
                        Eigen::RowVector4f b = fusedBV.row(i)/fusedConf_BV(i);
                        kpt.row((int)sl::BODY_34_BONES[i].second) = kpt.row((int)sl::BODY_34_BONES[i].first) + b.leftCols(3).normalized()*b(3);
                    }
                    if(isnan(kpt(30, 0)+kpt(28, 0))){
                        kpt.row(28)=Eigen::RowVector3f(keypoints[28].x, keypoints[28].y, keypoints[28].z);
                        kpt.row(29)=Eigen::RowVector3f(keypoints[29].x, keypoints[29].y, keypoints[29].z);
                        kpt.row(30)=Eigen::RowVector3f(keypoints[30].x, keypoints[30].y, keypoints[30].z);
                        kpt.row(31)=Eigen::RowVector3f(keypoints[31].x, keypoints[31].y, keypoints[31].z);
                    }
                    keypoints.clear();
                    for(int i=0;i<34;i++)
                    {
                        keypoints.push_back(sl::float3(kpt(i, 0), kpt(i, 1), kpt(i, 2)));
                        ofs_BV<<kpt.row(i)*100<<endl;
                    }
                    body.keypoint = keypoints;
                    fusedBody_BV.body_list.push_back(body);
                    
                    // dist[obj.id] = (Eigen::RowVector2f(keypoints[5].x, keypoints[5].y)-center).norm();
                    // diff[obj.id] = Eigen::RowVector2f(keypoints[5].x, keypoints[5].y)-center;
                }
                else
                {
                    ofs<<"p "<<-obj.id<<" "<<center(0)*100<<" "<<center(1)*100<<endl;
                    ofs_BV<<"p "<<-obj.id<<" "<<center(0)*100<<" "<<center(1)*100<<endl;
                }
            }
            // ofs<<frame_count<<"\t";
            // for(int i=0;i<6;i++)
            // {
            //     // if(dist.find(i)!=dist.end()) ofs<<dist[i];
            //     // ofs<<"\t";
            //     if(dist.find(i)!=dist.end()) ofs<<diff[i](0)<<"\t"<<diff[i](1)<<"\t";
            //     else ofs<<"\t\t";
            // }
            // ofs<<endl;
        }
        if(viewer.fused) viewer.updateData(camera_poses, camera_objects[frame_count][topView], fusedBody_BV, camera_pointClouds);    
        else viewer.updateData(camera_poses, camera_objects[frame_count], camera_bodies[frame_count], camera_pointClouds);    

        gl_viewer_available = viewer.isAvailable();
        cv::waitKey(60);
        if(viewer.stop){
            getchar();
            viewer.stop = false;
        }
        // if(pause)
        // {
        //     // key = (char)cv::waitKey(0);
        //     getchar();
        //     pause=  false;
        // }
        // else key = (char)cv::waitKey(100);

        // if (key == 'q') {
        //     quit = true;
        // }
        // else if (key == ']')
        // {
        //     cout<<111<<endl;
        //     pause = true;
        // }
        if(++frame_count==noFrames) break;
        // frame_count = ++frame_count==noFrames? 0: frame_count;
    }
    viewer.exit();
    ofs.close();
    return 0;
}

void convertAffineEigen2SL(const Eigen::Affine3f &inp, sl::Transform &out)
{
    Eigen::Vector3f T = inp.translation();
    Eigen::Quaternionf Q = Eigen::Quaternionf( inp.rotation() ); Q.normalize();
    sl::float3 T_sl(T.x(), T.y(), T.z());
    sl::float4 Q_sl(Q.x(), Q.y(), Q.z(), Q.w());
    out.setTranslation(T_sl);
    out.setOrientation(Q_sl);
}

void transformObjects(const Eigen::Affine3f& cam_pose, sl::Objects& camera_objects)
{
    for (auto& head:camera_objects.object_list) {
        for (auto& bb: head.head_bounding_box) {
            Eigen::Vector3f bb_tf = cam_pose * Eigen::Vector3f(bb.x, bb.y, bb.z+0.2);
            bb.x = bb_tf.x();
            bb.y = bb_tf.y();
            bb.z = bb_tf.z();
        }
        Eigen::Vector3f head_tf = cam_pose * Eigen::Vector3f(head.head_position.x, head.head_position.y, head.head_position.z);
        head.head_position.x = head_tf.x();
        head.head_position.y = head_tf.y();
        head.head_position.z = head_tf.z();
    }
}
void transformBodies(const Eigen::Affine3f& cam_pose, sl::Bodies& camera_bodies)
{
    for (auto& body:camera_bodies.body_list) {
        for (auto& kpt: body.keypoint) {
            Eigen::Vector3f kpt_tf = cam_pose * Eigen::Vector3f(kpt.x, kpt.y, kpt.z);
            kpt.x = kpt_tf.x();
            kpt.y = kpt_tf.y();
            kpt.z = kpt_tf.z();
        }
        for (auto& pt: body.bounding_box) {
            Eigen::Vector3f pt_tf = cam_pose * Eigen::Vector3f(pt.x, pt.y, pt.z);
            pt.x = pt_tf.x();
            pt.y = pt_tf.y();
            pt.z = pt_tf.z();
        }
    }
}

