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

std::queue<sensor_msgs::msg::Image::SharedPtr> rgbBuf;
std::queue<sensor_msgs::msg::Image::SharedPtr> depthBuf;
std::mutex mBufMutex;
std::condition_variable data_cond;

// void sync_callback(std::shared_ptr<ImageGrabber> igb) {
//     while (rclcpp::ok()) {
//         std::unique_lock<std::mutex> lock(mBufMutex);
//         data_cond.wait(lock, [] { return !rgbBuf.empty() && !depthBuf.empty(); });
//         auto rgb_msg = rgbBuf.front();
//         auto depth_msg = depthBuf.front();
//         rgbBuf.pop();
//         depthBuf.pop();
//         lock.unlock();
//         igb->grabRGBDImage(rgb_msg, depth_msg);
//     }
// }

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

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("orbslam3_rgbd_node");

    // Retrieve parameters
    node->declare_parameter("config_path", "");
    node->declare_parameter("vocab_path", "");

    std::string config_path = node->get_parameter("config_path").as_string();
    // std::string vocab_path = node->get_parameter("vocab_path").as_string();
    std::string vocab_path = "/home/sagar/Developer/ComputerVision/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt";

    bool showPangolin = true; // Show Pangolin window
    bool bEqual = false;

    // Publishers
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/slam", 10);
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/pointcloud", 10);

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

/*
// Old Code 

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
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char *argv[])
{
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
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/slam", 10);
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/pointcloud", 10);

    // Create SLAM system
    auto SLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::RGBD, showPangolin);
    auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "map");

    // Message Filters for Synchronization
    // message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub(node, "/camera/rgb/image_color", rmw_qos_profile_sensor_data);
    // message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub(node, "/camera/depth/image", rmw_qos_profile_sensor_data);
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    // auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub, depth_sub);
    // sync->registerCallback([igb](const sensor_msgs::msg::Image::SharedPtr rgb_msg, const sensor_msgs::msg::Image::SharedPtr depth_msg) {
    //     igb->grabRGBDImage(rgb_msg, depth_msg);
    // });

    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;
    rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, "/camera/rgb/image_color");
    depth_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, "/camera/depth/image");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&igb::grabRGBDImage, this);



    //-------------------
    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/rgb/image_color");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/depth/image");

    // syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    // syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);
    //-------------------

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}


*/