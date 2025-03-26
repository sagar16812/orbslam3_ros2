#ifndef IMAGE_GRABBER_RGBD_HPP
#define IMAGE_GRABBER_RGBD_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <condition_variable>
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
                std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name);

    void grabRGBDImage(const sensor_msgs::msg::Image::SharedPtr rgb_msg, const sensor_msgs::msg::Image::SharedPtr depth_msg);
    cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg);
    cv::Mat getDepthImage(const sensor_msgs::msg::Image::SharedPtr &depth_msg);
    // void virtual processImages();
    void processImages(const sensor_msgs::msg::Image::SharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::SharedPtr &depth_msg);
    void publishSE3fToOdom(const Sophus::SE3f& se3);
    void publishPointCloud(const std::vector<Eigen::Vector3f>& points);

    std::queue<std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>> rgbdBuf;
    std::queue<sensor_msgs::msg::Image::SharedPtr> rgbBuf;
    std::queue<sensor_msgs::msg::Image::SharedPtr> depthBuf;
    std::mutex mBufMutex;
    std::condition_variable data_cond;
    std::shared_ptr<ORB_SLAM3::System> mpSLAM;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    nav_msgs::msg::Odometry odom_msg_;
    std::shared_ptr<rclcpp::Node> rosNode_;
    const std::string tf_frame;
};

#endif // IMAGE_GRABBER_RGBD_HPP
