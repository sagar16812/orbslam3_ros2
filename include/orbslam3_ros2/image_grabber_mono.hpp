#ifndef IMAGE_GRABBER_HPP
#define IMAGE_GRABBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "include/System.h"  // Include the SLAM system header

#include <queue>
#include <mutex>
#include <memory>

class ImageGrabber : public std::enable_shared_from_this<ImageGrabber>
{
private:
    bool first_pose;
public:
    ImageGrabber();
    ImageGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe, 
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
    std::shared_ptr<rclcpp::Node> ros_node,const std::string camera_frame_name);

    void grabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg);

    void savePoseToFile(const Sophus::SE3f &pose, double sec, double nanosec);
    void virtual processImages();
    void publishSE3fToOdom(const Sophus::SE3f& se3);
    void publishPointCloud(const std::vector<Eigen::Vector3f>& points);

    std::queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::mutex mBufMutex;
    std::shared_ptr<ORB_SLAM3::System> mpSLAM;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    nav_msgs::msg::Odometry odom_msg_;
    std::shared_ptr<rclcpp::Node> rosNode_;
    const std::string tf_frame;
};

#endif // IMAGE_GRABBER_HPP
