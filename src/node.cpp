

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <map>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

#include "aruco.h"
#include "success_ros_msgs/DetectedMarkers.h"
#include "success_ros_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <mutex>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>

class AruCoProcessing
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber info_sub_;
    int process_rate_;
    ros::Publisher markers_pub_, poses_pub_;
    aruco::MarkerDetector detector_;
    float marker_size_meters_;
    std::string dictionary_name_;
    std::map<int, aruco::MarkerPoseTracker> tracker_map_;
    std::mutex image_mutex_;
    bool valid = false;
    cv::Mat image_;
    aruco::CameraParameters camera_info_;
    std_msgs::Header latest_header_;
    ros::Timer process_timer_;


  public:
    AruCoProcessing()
        : it_(nh_), nh_("~")
    {
        //load parameters
        if (!nh_.getParam("marker_dictionary", dictionary_name_))
            dictionary_name_ = "ARUCO_MIP_36h12";
        if (!nh_.getParam("marker_size", marker_size_meters_))
            marker_size_meters_ = 0.053;
        if (!nh_.getParam("rate", process_rate_))
            process_rate_ = 20;

        //set local parameters
        detector_.setDictionary(dictionary_name_);
        //subscribe to local channels
        image_sub_ = it_.subscribe("/image", 1,
                                       &AruCoProcessing::imageCb, this);
        info_sub_ = nh_.subscribe("/camera_info",1, &AruCoProcessing::infoCb, this);
        markers_pub_ = nh_.advertise<success_ros_msgs::DetectedMarkers>("markers", 1);
        poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("marker_poses", 1);
        ROS_INFO("ROS ARUCO: DICT: %s, size:%f", dictionary_name_.c_str(), marker_size_meters_);

        //create a timer that called to process the image
        //start this
        process_timer_ = nh_.createTimer(ros::Rate(process_rate_), &AruCoProcessing::processCB, this);
    }

    void infoCb(const sensor_msgs::CameraInfoConstPtr &cam_info)
    {

        float cam_data[9];
        if (cam_info->K.size() >= 9)
        {
            for (int i = 0; i < 9; i++)
            {
                cam_data[i] = cam_info->K[i];
            }
        }
        else
        {
            ROS_ERROR("Not enough matrix entries (9 needed) in cam_info->K. Undefined behavior");
        }
        cv::Mat cam_mat(3, 3, 5, cam_data);

        float distortion_data[] = {0, 0, 0, 0, 0};
        if (cam_info->D.size() >= 5)
        {
            for (int i = 0; i < 5; i++)
            {
                distortion_data[i] = cam_info->D[i];
            }
        }
        else
        {
            ROS_ERROR("No distortion model params in the camera info message -- has this camera been calibrated?");
        }
        cv::Mat distortian_mat(1, 5, 5, distortion_data);

        cv::Size cam_size(cam_info->width, cam_info->height);

        std::unique_lock<std::mutex> lock1(image_mutex_);
        camera_info_ = aruco::CameraParameters(cam_mat, distortian_mat, cam_size);
    }

    aruco::CameraParameters _convertCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info)
    {
        std::cout << cam_info->K[0] << std::endl;
        float cam_data[9];
        if (cam_info->K.size() >= 9)
        {
            for (int i = 0; i < 9; i++)
            {
                cam_data[i] = cam_info->K[i];
            }
        }
        else
        {
            ROS_ERROR("Not enough matrix entries (9 needed) in cam_info->K. Undefined behavior");
        }
        cv::Mat cam_mat(3, 3, 5, cam_data);

        float distortion_data[] = {0, 0, 0, 0, 0};
        if (cam_info->D.size() >= 5)
        {
            for (int i = 0; i < 5; i++)
            {
                distortion_data[i] = cam_info->D[i];
            }
        }
        else
        {
            ROS_ERROR("No distortion model params in the camera info message -- has this camera been calibrated?");
        }
        cv::Mat distortian_mat(1, 5, 5, distortion_data);

        cv::Size cam_size(cam_info->width, cam_info->height);

        return aruco::CameraParameters(cam_mat, distortian_mat, cam_size);
    }

    geometry_msgs::PoseStamped convertToStampedPose(std_msgs::Header header, cv::Mat& rvec, cv::Mat& tvec)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = header;

        //convert the rotation vector (axis angle format) to rotation matrix
        //modified from AruCo library
        float rx = rvec.ptr<float>(0)[0];
        float ry = rvec.ptr<float>(0)[1];
        float rz = rvec.ptr<float>(0)[2];
        float nsqa = rx*rx + ry*ry + rz*rz;
        float a = std::sqrt(nsqa);
        float i_a=a?1./a:0;
        tf::Quaternion rot_quaternion(tf::Vector3(rx * i_a, ry * i_a, rz * i_a), a);

        //Note: Rotate the axis -180 degrees around Z-axis. Such that y-axis points upwards in the image
        //tf::Quaternion axisCorrection;
        //axisCorrection.setEuler(0,0,-M_PI/2.0);
        //axisCorrection.setEuler(0,0,0);

        //correct the rotation matrix
        tf::Matrix3x3 rot_mat(rot_quaternion); //* axisCorrection);

        //the translation vector
        tf::Vector3 tv(
            tvec.at<float>(0),
            tvec.at<float>(1),
            tvec.at<float>(2));
        //correct the translation vector
        //tv = tv * tf::Matrix3x3(axisCorrection);

        tf::poseTFToMsg(tf::Transform(rot_mat, tv), pose.pose);

        return pose;
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        std::unique_lock<std::mutex> lock1(image_mutex_);
        //note to self, the incoming message have different encodings
        //kinect2 -> 8UC1 for mono
        //I'm not sure if the encoding will break the detector, but for now
        //we do not do any kind of convertions 
        image_ = cv_bridge::toCvCopy(msg, "mono8")->image;

        valid = true;
        latest_header_ = msg->header;
    }

    void processCB(const ros::TimerEvent &event)
    {
        //check to see if the process is running at the set speed (default 20Hz)
        if((1.0/process_rate_) < event.profile.last_duration.toSec()){
            ROS_WARN("Process is running behind set rate of %d real rate:%f",process_rate_, 1.0/(event.current_real - event.last_real).toSec());
        }

        cv::Mat image_raw;
        aruco::CameraParameters cam_parameter;
        std_msgs::Header header;
        //copy in the latest copy of the results
        {
            std::unique_lock<std::mutex> lock2(image_mutex_);
            if (!valid)
            {
                ROS_WARN("No valid Images found");
                return;
            }
            image_raw = image_;
            cam_parameter = camera_info_;
            header = latest_header_;
            valid = false;
        }
        //process the latest image 
        auto markers = detector_.detect(image_raw, cam_parameter, marker_size_meters_);
        ROS_DEBUG("Detected %lu markers", markers.size());

        //check if we see any markers
        if (markers.size() > 0)
        {
            success_ros_msgs::DetectedMarkers detected_markers_msgs;
            geometry_msgs::PoseArray poseArray;
            poseArray.header = header;
            std::vector<geometry_msgs::Pose> poses;
            for (auto &marker : markers)
            {
                auto pose = convertToStampedPose(header, marker.Rvec, marker.Tvec);
                success_ros_msgs::Marker marker_msg;
                marker_msg.id = marker.id;
                marker_msg.pose = pose;
                marker_msg.tag_dict_name = dictionary_name_;
                detected_markers_msgs.markers.push_back(marker_msg);
                poses.push_back(pose.pose);
            }
            markers_pub_.publish(detected_markers_msgs);
            poseArray.poses = poses;
            poses_pub_.publish(poseArray);
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_process_node");

    AruCoProcessing pc;
    ROS_INFO("Spinning");
    ros::spin();

}
