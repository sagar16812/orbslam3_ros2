#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/qos.hpp>
#include <rmw/types.h>  

#include "include/System.h"  // Include the SLAM system header

#include "orbslam3_ros2/image_grabber.hpp" // Change the "orbslam3_ros2" with your package name

#include <queue>
#include <mutex>
#include <thread>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("orbslam3_ros2");
    
    //std::string config_path = node->get_parameters("config_path").as_string();
    //std::string vocab_path = node->get_parameters("vocab_path").as_string();
    
    std::string config_path = "/home/sagar/Developer/ComputerVision/SLAM/slam_ws/src/orbslam3_ros2/config/camera_and_slam_settings.yaml";
    std::string vocab_path = "/home/sagar/Developer/ComputerVision/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    
    //bool showPangolin = false ; // true If you want to spone the Pangolin window with pose estimation drawed
    bool showPangolin = true ; // true If you want to spone the Pangolin window with pose estimation drawed
    bool bEqual = false;

    //--------------------------Publishers--------------------------

    // Publish odom message from SE3
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/slam", 10);
    
    // Publish 3D Point-Cloud
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/pointcloud", 10);

    // Create SLAM system and ImageGrabber
    auto SLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::MONOCULAR, showPangolin);
    auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "oak-d_frame");

    // Creating Image subscription
    std::string imgTopicName = "/myrobot/image_raw" ;
    // Subscribe to the camera image topic
    auto sub_img0 = node->create_subscription<sensor_msgs::msg::Image>(
        imgTopicName, 5, [igb](const sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(rclcpp::get_logger("orbslam3_ros2"), "Received an image!"); igb->grabImage(msg); });

    // Start processing images in a separate thread
    std::thread image_thread(&ImageGrabber::processImages, igb);

    // Run the ROS node
    rclcpp::spin(node);
    std::cout << "Node stop to spinning!" << std::endl;

    // Shutdown the node and wait for the thread to complete
    rclcpp::shutdown();
    image_thread.join();

    return 0;
}
