#include "orbslam3_ros2/image_grabber_stereo.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>

ImageGrabber::ImageGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
    std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name)
    : mpSLAM(pSLAM), mbClahe(bClahe), odom_pub_(rospub), cloud_pub_(cloud_pub),
      rosNode_(ros_node), tf_frame(camera_frame_name) {
    odom_msg_.header.frame_id = tf_frame;
    odom_msg_.child_frame_id = "odom";
}

// cv::Mat ImageGrabber::getImageLeft(const sensor_msgs::msg::Image::SharedPtr &msgLeftImg) {
//     try {
//         cv::Mat image = cv_bridge::toCvCopy(msgLeftImg, "bgr8")->image;
//         if (mbClahe) {
//             cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
//             mClahe->apply(image, image);
//         }
//         return image;
//     } catch (cv_bridge::Exception &e) {
//         RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
//         return cv::Mat();
//     }
// }

// cv::Mat ImageGrabber::getImageRight(const sensor_msgs::msg::Image::SharedPtr &msgRightImg) {
//     try {
//         return cv_bridge::toCvCopy(msgRightImg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
//     } catch (cv_bridge::Exception &e) {
//         RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
//         return cv::Mat();
//     }
// }
cv::Mat ImageGrabber::getImageLeft(const sensor_msgs::msg::Image::SharedPtr &msgLeftImg) {
    try {
        cv::Mat image;
        
        // Check if the input image is already mono8 (grayscale)
        if (msgLeftImg->encoding == sensor_msgs::image_encodings::MONO8) {
            image = cv_bridge::toCvCopy(msgLeftImg, sensor_msgs::image_encodings::MONO8)->image;
        } else {
            // Convert to grayscale
            image = cv_bridge::toCvCopy(msgLeftImg, sensor_msgs::image_encodings::BGR8)->image;
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        }

        // Apply CLAHE if enabled
        if (mbClahe) {
            mClahe->apply(image, image);
        }

        return image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

cv::Mat ImageGrabber::getImageRight(const sensor_msgs::msg::Image::SharedPtr &msgRightImg) {
    try {
        cv::Mat image;
        
        // Check if the input image is already mono8 (grayscale)
        if (msgRightImg->encoding == sensor_msgs::image_encodings::MONO8) {
            image = cv_bridge::toCvCopy(msgRightImg, sensor_msgs::image_encodings::MONO8)->image;
        } else {
            // Convert to grayscale
            image = cv_bridge::toCvCopy(msgRightImg, sensor_msgs::image_encodings::BGR8)->image;
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        }

        // Apply CLAHE if enabled
        if (mbClahe) {
            mClahe->apply(image, image);
        }

        return image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

void ImageGrabber::processImages(const sensor_msgs::msg::Image::SharedPtr &msgLeftImg,
    const sensor_msgs::msg::Image::SharedPtr &msgRightImg) {
    cv::Mat imLeft = getImageLeft(msgLeftImg);
    cv::Mat imRight = getImageRight(msgRightImg);

    if (imLeft.empty() || imRight.empty()) return;
    // std::cout << "Left image type: " << imLeft.type() << ", Channels: " << imLeft.channels() << std::endl;
    // std::cout << "Right image type: " << imRight.type() << ", Channels: " << imRight.channels() << std::endl;


    Sophus::SE3f pose = mpSLAM->TrackStereo(imLeft, imRight,
    msgLeftImg->header.stamp.sec + 1e-9 * msgLeftImg->header.stamp.nanosec);

    std::vector<ORB_SLAM3::MapPoint*> mapPoints = mpSLAM->GetTrackedMapPoints();

    std::vector<Eigen::Vector3f> point_cloud;
    for (auto p : mapPoints) {
    if (p && !p->isBad()) {
    Eigen::Vector3f pos = p->GetWorldPos();
    point_cloud.emplace_back(pos[0], pos[1], pos[2]);
    }
    }
    publishSE3fToOdom(pose);
    publishPointCloud(point_cloud);
}

void ImageGrabber::publishSE3fToOdom(const Sophus::SE3f& se3)
{
    odom_msg_.header.stamp = rosNode_->get_clock()->now();

    odom_msg_.pose.pose.position.x = -se3.translation().z();   // Z_OCV → X_ROS
    odom_msg_.pose.pose.position.y = se3.translation().x();  // -X_OCV → Y_ROS
    odom_msg_.pose.pose.position.z = se3.translation().y();  // -Y_OCV → Z_ROS

    // Convert ORB-SLAM3 quaternion to ROS2 (handle different coordinate frames)
    Eigen::Quaternionf q_ocv(se3.unit_quaternion());
    
    Eigen::Quaternionf q_conversion(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));

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

void ImageGrabber::publishPointCloud(const std::vector<Eigen::Vector3f>& points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    //cloud_msg.header.stamp = rosNode_->get_clock()->now();
    cloud_msg.header.frame_id = "map";  // Set a fixed frame for the map 
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
        *iter_x = point.z();  // Z_OCV → X_ROS
        *iter_y = -point.x(); // -X_OCV → Y_ROS
        *iter_z = -point.y(); // -Y_OCV → Z_ROS
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    cloud_pub_->publish(cloud_msg);
}