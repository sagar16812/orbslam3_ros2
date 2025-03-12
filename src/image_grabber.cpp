#include "orbslam3_ros2/image_grabber.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>

#include <sophus/se3.hpp>

ImageGrabber::ImageGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
    std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name)
    : mpSLAM(pSLAM), mbClahe(bClahe), first_pose(true), odom_pub_(rospub), cloud_pub_(cloud_pub),
      rosNode_(ros_node), tf_frame(camera_frame_name){
        // odom_msg_.header.frame_id = tf_frame;
        // odom_msg_.child_frame_id = "odom";
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_link";

        // odom_msg_.pose.pose.position.x = 0.0;
        // odom_msg_.pose.pose.position.y = 0.0;
        // odom_msg_.pose.pose.position.z = 0.0;
        
        // odom_msg_.pose.pose.orientation.x = 0.0;
        // odom_msg_.pose.pose.orientation.y = 0.0;
        // odom_msg_.pose.pose.orientation.z = 0.0;
        // odom_msg_.pose.pose.orientation.w = 0.0;
    }

void ImageGrabber::grabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    img0Buf.push(msg);
}

cv::Mat ImageGrabber::getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        if (mbClahe)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            mClahe->apply(image, image);
        }
        return image;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

/*
void ImageGrabber::processImages()
{
    while (rclcpp::ok())
    {
        sensor_msgs::msg::Image::SharedPtr img_msg;
        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            if (img0Buf.empty())
                continue;
            img_msg = img0Buf.front();
            img0Buf.pop();
        }
        cv::Mat image = getImage(img_msg);
        if (image.empty())
            continue;

        Sophus::SE3f pose;
        std::vector<Eigen::Vector3f> point_cloud;
        pose = mpSLAM->TrackMonocular(image, img_msg->header.stamp.sec + 1e-9 * img_msg->header.stamp.nanosec);
        point_cloud = mpSLAM->GetCurrentPointCloud();

        publishSE3fToOdom(pose);
        publishPointCloud(point_cloud);
    }
}
*/

void ImageGrabber::processImages()
{
    while (rclcpp::ok())
    {
        sensor_msgs::msg::Image::SharedPtr img_msg;
        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            if (img0Buf.empty())
                continue;
            img_msg = img0Buf.front();
            img0Buf.pop();
        }
        cv::Mat image = getImage(img_msg);
        if (image.empty())
            continue;

        Sophus::SE3f pose;
        std::vector<Eigen::Vector3f> point_cloud;

        // Track the image and get the camera pose
        pose = mpSLAM->TrackMonocular(image, img_msg->header.stamp.sec + 1e-9 * img_msg->header.stamp.nanosec);

        // Get the 3D map points from the SLAM system
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = mpSLAM->GetTrackedMapPoints();

        // Convert ORB-SLAM3 MapPoints to Eigen::Vector3f for ROS2 point cloud
        for (auto p : mapPoints)
        {
            if (p && !p->isBad()) // Ensure valid points
            {
                Eigen::Vector3f pos = p->GetWorldPos(); // Get 3D position
                point_cloud.emplace_back(pos[0], pos[1], pos[2]);
            }
        }

        // Publish pose and point cloud
        publishSE3fToOdom(pose);
        publishPointCloud(point_cloud);
    }
}

void ImageGrabber::publishSE3fToOdom(const Sophus::SE3f& se3)
{
    odom_msg_.header.stamp = rosNode_->get_clock()->now();
    //odom_msg_.header.frame_id = tf_frame;
    //odom_msg_.child_frame_id = "base_link"
    // odom_msg_.header.frame_id = "odom";

    // odom_msg_.pose.pose.position.x = se3.translation().x();
    // odom_msg_.pose.pose.position.y = se3.translation().y();
    // odom_msg_.pose.pose.position.z = se3.translation().z();
    // Eigen::Quaternionf q(se3.unit_quaternion());
    // odom_msg_.pose.pose.orientation.x = q.x();
    // odom_msg_.pose.pose.orientation.y = q.y();
    // odom_msg_.pose.pose.orientation.z = q.z();
    // odom_msg_.pose.pose.orientation.w = q.w();
    // odom_pub_->publish(odom_msg_);

    // Convert ORB-SLAM3 coordinate system to ROS2
    // Print before conversion
    std::cout << "Before Conversion (SE3f):" << std::endl;
    // std::cout << "Translation: " << se3.translation().transpose() << std::endl;
    std::cout << "x: " << se3.translation().x();
    std::cout << " y: " << se3.translation().y();
    std::cout << " z: " << se3.translation().z() << std::endl;
    // std::cout << "Rotation Matrix:\n" << se3.rotationMatrix() << std::endl;

    odom_msg_.pose.pose.position.x = se3.translation().z();   // Z_OCV → X_ROS
    odom_msg_.pose.pose.position.y = -se3.translation().x();  // -X_OCV → Y_ROS
    odom_msg_.pose.pose.position.z = -se3.translation().y();  // -Y_OCV → Z_ROS

    // Print after conversion
    std::cout << "After Conversion (ROS2):" << std::endl;
    std::cout << "x: " << odom_msg_.pose.pose.position.x;
    std::cout << " y: " << odom_msg_.pose.pose.position.y;
    std::cout << " z: " << odom_msg_.pose.pose.position.z << std::endl;

    // Convert ORB-SLAM3 quaternion to ROS2 (handle different coordinate frames)
    Eigen::Quaternionf q_ocv(se3.unit_quaternion());

    // Apply the transformation: q_ros = q_ocv * q_conversion
    Eigen::Quaternionf q_conversion(0.5, -0.5, 0.5, 0.5);  // Rotation to align frames

    // Correct rotation to align ORB-SLAM3 with ROS2 REP-105 (90° rotation around X-axis)
    // Eigen::Quaternionf q_conversion(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));

    Eigen::Quaternionf q_ros = q_conversion * q_ocv;

    odom_msg_.pose.pose.orientation.x = q_ros.x();
    odom_msg_.pose.pose.orientation.y = q_ros.y();
    odom_msg_.pose.pose.orientation.z = q_ros.z();
    odom_msg_.pose.pose.orientation.w = q_ros.w();

    // --- Set Covariance Values ---
    double position_variance = 0.01;  // Adjust based on your SLAM system's accuracy
    double orientation_variance = 0.02;

    for (int i = 0; i < 36; i++) odom_msg_.pose.covariance[i] = 0.0;

    odom_msg_.pose.covariance[0] = position_variance;  // x
    odom_msg_.pose.covariance[7] = position_variance;  // y
    odom_msg_.pose.covariance[14] = position_variance; // z

    odom_msg_.pose.covariance[21] = orientation_variance; // roll
    odom_msg_.pose.covariance[28] = orientation_variance; // pitch
    odom_msg_.pose.covariance[35] = orientation_variance; // yaw
    // --------------------------------
    
    odom_pub_->publish(odom_msg_);
}

// void ImageGrabber::publishPointCloud(const std::vector<Eigen::Vector3f>& points)
// {
//     sensor_msgs::msg::PointCloud2 cloud_msg;
//     cloud_msg.header.stamp = rosNode_->get_clock()->now();
//     cloud_msg.header.frame_id = tf_frame;
//     cloud_msg.height = 1;
//     cloud_msg.width = points.size();
//     cloud_msg.is_dense = false;
//     cloud_msg.is_bigendian = false;

//     sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
//     modifier.setPointCloud2FieldsByString(1, "xyz");
//     modifier.resize(points.size());

//     sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

//     for (const auto& point : points)
//     {
//         *iter_x = point.x();
//         *iter_y = point.y();
//         *iter_z = point.z();
//         ++iter_x;
//         ++iter_y;
//         ++iter_z;
//     }

//     cloud_pub_->publish(cloud_msg);
// }

void ImageGrabber::publishPointCloud(const std::vector<Eigen::Vector3f>& points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    //cloud_msg.header.stamp = rosNode_->get_clock()->now();
    // cloud_msg.header.frame_id = "map";  // Set a fixed frame for the map
    //cloud_msg.header.frame_id = tf_frame;  // Set a fixed frame for the map 
    cloud_msg.header.frame_id = "base_link";  // Set a fixed frame for the map 
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& point : points)
    {
        // *iter_x = point.x();
        // *iter_y = point.y();
        // *iter_z = point.z();
        // ++iter_x;
        // ++iter_y;
        // ++iter_z;

        *iter_x = point.z();  // Z_OCV → X_ROS
        *iter_y = -point.x(); // -X_OCV → Y_ROS
        *iter_z = -point.y(); // -Y_OCV → Z_ROS
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    cloud_pub_->publish(cloud_msg);
}