#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
//#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
//#include <Eigen/SVD>
using namespace std;
using namespace cv;
//using namespace aruco;
using namespace Eigen;
using namespace message_filters;

#define num_marker_used 2
#define tlmid 0
//#define trmid 1
//#define blmid 2
#define brmid 3

// #define ransac_required_sample 10
// #define ransac_inlayer_dist 0.05
// #define ransac_inlayer_ori 0.9

#define required_sample 100

float MarkerSize = 0.15;
//int query_id = 23;
//aruco::CameraParameters CamParam;
cv::Mat K, D;

//MarkerDetector MDetector;
//vector<Marker> Markers;

Matrix<vector<Vector3d>,num_marker_used,1> marker_pos;
Matrix<vector<Quaterniond>,num_marker_used,1> marker_ori;
int marker_cnt = 0;
bool ransac_flag = 0;
bool estimate_complete_flag = 0;

Quaterniond Q_mg;

ros::Publisher gap_pose_pub;
//ros::Publisher centroid_pub;


// void process_ransac(){

//     vector<Vector3d> marker_pos_sample(ransac_required_sample, Vector3d(0,0,0));
//     vector<Matrix3d> marker_ori_sample(2 * ransac_required_sample, Vector3d(0,0,0));
    
//     vector<int> inlayer_pos_cnt(ransac_required_sample, 0);
//     vector<int> inlayer_ori_cnt(ransac_required_sample, 0);
//     vector<Vector3d> marker_pos_sum(ransac_required_sample, Vector3d(0,0,0));
//     vector<Matrix3d> marker_ori_sum(ransac_required_sample, Vector3d(0,0,0));
    
//     for(int i=0; i<ransac_required_sample; i++){
//         Vector2i rand_num (rand() % marker_pos(0).size(), rand() % marker_pos(1).size());
//         marker_pos_sample[i] = ((marker_pos(0))[rand_num(0)] + (marker_pos(1))[rand_num(1)]) * 0.5;
//         marker_ori_sample[2*i] = ((marker_ori(0))[rand_num(0)]).toRotationMatrix();
//         marker_ori_sample[2*i+1] = ((marker_ori(1))[rand_num(1)]).toRotationMatrix();

//         marker_pos(0).erase(marker_pos(0).begin()+rand_num(0));
//         marker_pos(1).erase(marker_pos(1).begin()+rand_num(1));

//         marker_ori(0).erase(marker_ori(0).begin()+rand_num(0));
//         marker_ori(1).erase(marker_ori(1).begin()+rand_num(1));
//     }

//     for(int i=0; i<marker_pos_sample.size(); i++){

//         inlayer_pos_cnt[i]++;
//         marker_pos_sum[i] += marker_pos_sample[i];

//        for(int j=i+1; j<marker_pos_sample.size(); j++){
//             if((marker_pos_sample[j] - marker_pos_sample[i]).norm() < ransac_inlayer_dist){
//                 inlayer_pos_cnt[i]++;
//                 inlayer_pos_cnt[j]++;

//                 marker_pos_sum[i] += marker_pos_sample[j];
//                 marker_pos_sum[j] += marker_pos_sample[i];
//             }
//        }
//     }

//     for(int i=0; i<marker_ori_sample.size(); i++){

//         inlayer_ori_cnt[i]++;
//         marker_ori_sum[i] += marker_ori_sample[i];

//        for(int j=i+1; j<marker_ori_sample.size(); j++){
//             if(marker_ori_sample[j].dot(marker_ori_sample[i]) > ransac_inlayer_ori){
//                 inlayer_ori_cnt[i]++;
//                 inlayer_ori_cnt[j]++;

//                 marker_ori_sum[i] += marker_ori_sample[j];
//                 marker_ori_sum[j] += marker_ori_sample[i];
//             }
//        }
//     }

//     int max_pos_idx = 0, max_ori_idx = 0;
//     int max_pos_cnt = 0, max_ori_cnt = 0;
//     for(int i=0; i<marker_pos_sample.size(); i++){
//         if(inlayer_pos_cnt[i] > max_pos_cnt){
//             max_pos_cnt = inlayer_pos_cnt[i];
//             max_pos_idx = i;
//         }
//     }
//     for(int i=0; i<marker_ori_sample.size(); i++){
//         if(inlayer_ori_cnt[i] > max_ori_cnt){
//             max_ori_cnt = inlayer_ori_cnt[i];
//             max_ori_idx = i;
//         }
//     }
// }


Eigen::Quaterniond mean_quaternion(Matrix<vector<Quaterniond>,num_marker_used,1>& marker_ori){

    Matrix4d quater_sum = Matrix4d::Zero();
    int cnt = 0;

    for(int i=0; i<num_marker_used; i++){
        for(auto it=marker_ori(i,0).begin(); it!=marker_ori(i,0).end(); it++){
            Vector4d temp_quater(it->w(),it->x(),it->y(),it->z());
            quater_sum += (temp_quater * temp_quater.transpose());
            cnt++;
        }
    }

    quater_sum /= cnt;

    EigenSolver<Matrix4d> es(quater_sum);
    Vector4d ev(es.eigenvalues().row(0).norm(), es.eigenvalues().row(1).norm(), es.eigenvalues().row(2).norm(), es.eigenvalues().row(3).norm());
    double max_ev = 0;
    int max_ev_idx = -1;

    for (int i = 0; i <ev.rows(); ++i){
        if(abs(ev(i)) > max_ev){
            max_ev_idx = i;
            max_ev = abs(ev(i));
        }
    }

    Quaterniond result(es.eigenvectors().col(max_ev_idx).row(0).norm(), es.eigenvectors().col(max_ev_idx).row(1).norm(), es.eigenvectors().col(max_ev_idx).row(2).norm(), es.eigenvectors().col(max_ev_idx).row(3).norm());

    return result;

}

Eigen::Vector3d mean_pos_diag(Matrix<vector<Vector3d>,2,1> marker_pos, int marker_cnt){

    Vector3d mean_pos(0,0,0);
    for(int i = 0; i < marker_cnt; i++){
        mean_pos += (marker_pos(0,0))[i];
        mean_pos += (marker_pos(1,0))[i];
    }

    mean_pos /= (2.0*marker_cnt);

    return mean_pos;
}


void img_callback(const sensor_msgs::ImageConstPtr &img_msg, const geometry_msgs::PoseStampedConstPtr &camera_pose_msg)
{
    //ROS_INFO("Received image!");

    if(estimate_complete_flag) return;

    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    Mat frame = bridge_ptr->image;

    Vector3d camera_position(camera_pose_msg->pose.position.x, camera_pose_msg->pose.position.y, camera_pose_msg->pose.position.z);
    Quaterniond camera_orientation(camera_pose_msg->pose.orientation.w, camera_pose_msg->pose.orientation.x, camera_pose_msg->pose.orientation.y, camera_pose_msg->pose.orientation.z);
    camera_orientation.normalize();

    //MDetector.detect(frame, Markers);

    vector< int > markerIds; 
    vector< vector<Point2f> > markerCorners, rejectedCandidates; 
    //cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    

    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);//, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    
    vector< Vec3d > rvecs, tvecs;

    if(markerIds.size()>0){
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.15, K, D, rvecs, tvecs);
        cv::aruco::drawAxis(frame, K, D, rvecs, tvecs, 0.1);        
    }
    
    for(int i=0; i<markerIds.size(); i++){ 
        
        Vec3d rvec = rvecs[i];
        Vec3d tvec = tvecs[i];
        Mat R;
        cv::Rodrigues(rvec, R);

        Matrix3d R_eigen;
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++) {
                R_eigen(j,k) = R.at<double>(j,k);
            }

        Vector3d T_eigen;
        for(int j=0; j<3; j++) T_eigen(j) = tvec(j);


        //std::cout<<"detected ori:\n"<<R_eigen<<endl<<endl;
        //std::cout<<"detected translation:\n"<<T_eigen<<endl<<endl<<endl<<endl;

        Quaterniond Q_cm;
        Q_cm = R_eigen;
        Q_cm.normalize();

        int position_to_push = -1;

        switch(markerIds[i]){

            case tlmid: position_to_push = 0; break;
            //case trmid: position_to_push = 1; break;
            //case blmid: position_to_push = 2; break;
            case brmid: position_to_push = 1; break;

            default: break;

        }

        Vector3d temp_marker_pos = camera_orientation.toRotationMatrix() * T_eigen + camera_position;
        Quaterniond temp_marker_ori = camera_orientation * Q_cm * Q_mg;

        std::cout<<"marker_pos:\n"<<temp_marker_pos<<endl;
        std::cout<<"marker_ori:\n"<<temp_marker_ori.toRotationMatrix()<<endl;


        marker_pos(position_to_push).push_back(temp_marker_pos);
        marker_ori(position_to_push).push_back(temp_marker_ori);

    }

    marker_cnt = min(marker_pos(0).size(),marker_pos(1).size());


    // if(marker_cnt >= 2*ransac_required_sample && !ransac_flag){
    // }

    if(marker_cnt >= required_sample && !estimate_complete_flag){
        Quaterniond final_ori = mean_quaternion(marker_ori);
        final_ori.normalize();
        Vector3d final_pos = mean_pos_diag(marker_pos, marker_cnt);

        geometry_msgs::PoseStamped gap_pose;

        gap_pose.header = camera_pose_msg->header;
        gap_pose.header.frame_id = "world";

        gap_pose.pose.position.x = final_pos.x();
        gap_pose.pose.position.y = final_pos.y();
        gap_pose.pose.position.z = final_pos.z();

        gap_pose.pose.orientation.w = final_ori.w();
        gap_pose.pose.orientation.x = final_ori.x();
        gap_pose.pose.orientation.y = final_ori.y();
        gap_pose.pose.orientation.z = final_ori.z();

        for(int i = 0; i < 3; i++)
            gap_pose_pub.publish(gap_pose);
    }


    // for (unsigned int i = 0; i < Markers.size(); i++)
    // {

    //     //ROS_WARN("Got one marker!");
    //     //cout << Markers[i] << endl;
     
    //     Markers[i].calculateExtrinsics(MarkerSize, CamParam, false);
    //     Markers[i].draw(frame, Scalar(0,0,255), 2);
    //     aruco::CvDrawingUtils::draw3dAxis(frame, Markers[i], CamParam);

    //     Mat rvec = Markers[i].Rvec;
    //     Mat tvec = Markers[i].Tvec;
    //     Mat R;
    //     Rodrigues(rvec, R);

    //     Matrix3d R_eigen;
    //     for(int j=0;j<3;j++)
    //         for(int k=0;k<3;k++) {
    //             R_eigen(j,k) = R.at<float>(j,k);
    //         }

    //     Vector3d T_eigen;
    //     for(int j=0; j<3; j++) T_eigen(j) = tvec.at<float>(j,0);

    //     T_eigen = R_eigen.transpose() * (-T_eigen);

    //     Quaterniond Q;
    //     Q = R_eigen.transpose();
    //     Q.normalize();

    //     int position_to_push = -1;

    //     switch(Markers[i].id){

    //         case tlmid: position_to_push = 0; break;
    //         case trmid: position_to_push = 1; break;
    //         case blmid: position_to_push = 2; break;
    //         case brmid: position_to_push = 3; break;

    //         default: break;

    //     }

    //     Vector3d temp_marker_pos = camera_orientation.toRotationMatrix() * T_eigen + camera_position;
    //     Quaterniond temp_marker_ori = camera_orientation * Q * Q_mg;

    //     std::cout<<"marker_pos:\n"<<temp_marker_pos<<endl;
    //     std::cout<<"marker_ori:\n"<<temp_marker_ori.toRotationMatrix()<<endl;


    //     marker_pos(position_to_push).push_back(temp_marker_pos);
    //     marker_ori(position_to_push).push_back(temp_marker_ori);

        
    //     nav_msgs::Odometry odom_marker;

    //     odom_marker.header.stamp = img_msg->header.stamp;
    //     odom_marker.header.frame_id = "camera"; //TODO: ???
    //     odom_marker.pose.pose.position.x = tvec.at<float>(0,0);
    //     odom_marker.pose.pose.position.y = tvec.at<float>(1,0);
    //     odom_marker.pose.pose.position.z = tvec.at<float>(2,0);

    //     //cout << "tvec:  " << tvec.at<float>(0,0) << endl << tvec.at<float>(1,0) << endl << tvec.at<float>(2,0) << endl << "====" << endl;

    //     odom_marker.pose.pose.orientation.w = Q.w();
    //     odom_marker.pose.pose.orientation.x = Q.x();
    //     odom_marker.pose.pose.orientation.y = Q.y();
    //     odom_marker.pose.pose.orientation.z = Q.z();

        //Add id to the message
    //}
    //Mat frame_undistort;
    //cv::undistort(frame, frame_undistort, K, D);
    imshow("usb_image", frame);
    waitKey(5);
}


void image_callback(const sensor_msgs::ImageConstPtr &img_msg){
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    Mat frame = bridge_ptr->image;

    Mat frame_undistort;
    undistort(frame, frame_undistort, K, D);
    imshow("usb_image", frame_undistort);
    waitKey(5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detector");
    ros::NodeHandle nh("~");
    string cam_cal;
    nh.getParam("cam_cal_file", cam_cal);
    //CamParam.readFromXMLFile(cam_cal);
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    cout<<K<<endl;
    cout<<D<<endl;

    Matrix3d R_mg;
    R_mg<<  0,-1,0,
            0,0,1,
            -1,0,0;
    Q_mg = R_mg;
    Q_mg.normalize();

    
    gap_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gap_pose", 10);
    //pub_ar_odom = nh.advertise<nav_msgs::Odometry>("/detected_markers", 10);
    ros::Subscriber sub_image = nh.subscribe("/djiros/image", 1, image_callback);
    //ros::Subscriber sub_cam_pose = nh.subscribe("camera_pose", 1, img_callback);
    

    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, "image", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_cam_pose(nh, "camera_pose", 1);

    typedef sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_img, sub_cam_pose);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));
    
    //cv::namedWindow("Marker Detector", 1);
    ros::spin();
}
