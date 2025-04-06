#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>
#include "include/System.h"  // ORB-SLAM3 System
#include "orbslam3_ros2/image_grabber_rgbd.hpp" // Adjust package name if needed
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
std::queue<sensor_msgs::msg::Image::SharedPtr> rgbBuf;
std::queue<sensor_msgs::msg::Image::SharedPtr> depthBuf;
std::mutex mBufMutex;
std::condition_variable data_cond;

void sync_callback(std::shared_ptr<ImageGrabber> igb) {
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lock(mBufMutex);
        data_cond.wait(lock, [] { return !rgbBuf.empty() && !depthBuf.empty(); });

        auto rgb_msg = rgbBuf.front();
        auto depth_msg = depthBuf.front();
        rgbBuf.pop();
        depthBuf.pop();
        lock.unlock();

        igb->processImages(rgb_msg, depth_msg); // Pass directly for processing
    }
}

// Function to broadcast static transform
void publish_static_transform(std::shared_ptr<rclcpp::Node> node)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(node);
    geometry_msgs::msg::TransformStamped static_transform;

    static_transform.header.stamp = node->now();
    static_transform.header.frame_id = "map";   // The reference frame
    static_transform.child_frame_id = "odom"; // Your camera frame    
    static_broadcaster.sendTransform(static_transform);
    //RCLCPP_INFO(node->get_logger(), "Published static transform: map -> odom");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("orbslam3_rgbd_node");

    // Retrieve parameters
    node->declare_parameter("config_path", "");
    node->declare_parameter("vocab_path", "");

    std::string config_path = node->get_parameter("config_path").as_string();
    std::string vocab_path = node->get_parameter("vocab_path").as_string();

    bool showPangolin = true; // Show Pangolin window
    bool bEqual = false;

    // Publishers
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/slam/odometry", 10);
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/pointcloud", 10);

    // Publish static transform
    publish_static_transform(node);
    
    // Create SLAM system
    auto SLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::RGBD, showPangolin);
    auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "map");

    // RGB subscription
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/rgb/image_color", rclcpp::SensorDataQoS(),
        [](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mBufMutex);
            rgbBuf.push(msg);
            data_cond.notify_one();
        });

    // Depth subscription
    auto depth_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image", rclcpp::SensorDataQoS(),
        [](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mBufMutex);
            depthBuf.push(msg);
            data_cond.notify_one();
        });

    // Start synchronization thread
    std::thread sync_thread(sync_callback, igb);

    // Spin the node
    rclcpp::spin(node);

    // Cleanup
    sync_thread.join();
    rclcpp::shutdown();
    return 0;
}