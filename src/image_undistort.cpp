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

cv::Mat K, D;

ros::Publisher image_undistort_pub;


void image_callback(const sensor_msgs::ImageConstPtr &img_msg){
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    Mat frame = bridge_ptr->image;

    Mat frame_undistort;
    undistort(frame, frame_undistort, K, D);
    imshow("usb_image", frame_undistort);
    sensor_msgs::Image img_undistort;
    cv_bridge::CvImage img_bridge;
    img_bridge = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::RGB8, frame_undistort);
    img_bridge.toImageMsg(img_undistort);
    image_undistort_pub.publish(img_undistort);
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

    ros::Subscriber sub_image = nh.subscribe("/djiros/image", 1, image_callback);
    image_undistort_pub = nh.advertise<sensor_msgs::Image>("image_undistort", 10);
    

    
    //cv::namedWindow("Marker Detector", 1);
    ros::spin();
}
