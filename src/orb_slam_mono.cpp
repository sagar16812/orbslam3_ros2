#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/qos.hpp>
#include <rmw/types.h>  

#include "include/System.h"  // Include the SLAM system header

#include "orbslam3_ros2/image_grabber_mono.hpp" // Change the "orbslam3_ros2" with your package name

#include <queue>
#include <mutex>
#include <thread>

//-----New-----
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
//-----New-----


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


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("orbslam3_ros2");

    // Retrieve parameters
    node->declare_parameter("config_path", "");  // Declare with default empty value
    node->declare_parameter("vocab_path", ""); 
        
    std::string config_path = node->get_parameter("config_path").as_string();
    std::string vocab_path = node->get_parameter("vocab_path").as_string();
    
    bool showPangolin = true ; // true If you want to spone the Pangolin window with pose estimation drawed
    bool bEqual = false;

    //--------------------------Publishers--------------------------

    // Publish odom message from SE3
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/slam/odometry", 10);
    
    // Publish 3D Point-Cloud
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/pointcloud", 10);

    // Publish static transform
    publish_static_transform(node);

    // Create SLAM system and ImageGrabber
    auto SLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::MONOCULAR, showPangolin);
    // auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "oak-d_frame");
    
    
    auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "map");
    // auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "odom");

    // Creating Image subscription
    std::string imgTopicName = "/camera/rgb/image_color" ;
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
