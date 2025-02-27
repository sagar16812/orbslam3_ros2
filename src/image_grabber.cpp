#include "orbslam3_ros2/image_grabber.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sophus/se3.hpp>

// Class Constructor
ImageGrabber::ImageGrabber() : mbClahe(false), first_pose(true) {}

ImageGrabber::ImageGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe, 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name)
    : mpSLAM(pSLAM), mbClahe(bClahe), first_pose(true),
    odom_pub_(rospub), rosNode_(ros_node),
    tf_frame(camera_frame_name) 
    {
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = tf_frame;

        odom_msg_.pose.pose.position.x = 0.0;
        odom_msg_.pose.pose.position.y = 0.0;
        odom_msg_.pose.pose.position.z = 0.0;
        
        odom_msg_.pose.pose.orientation.x = 0.0;
        odom_msg_.pose.pose.orientation.y = 0.0;
        odom_msg_.pose.pose.orientation.z = 0.0;
        odom_msg_.pose.pose.orientation.w = 0.0;
    }

// Image Handling
void ImageGrabber::grabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (!img0Buf.empty())
        img0Buf.pop();  // Remove the oldest image to process the latest one
    img0Buf.push(img_msg);
}

cv::Mat ImageGrabber::getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    // Convert the ROS image message to a cv::Mat object
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
    return cv_ptr->image.clone();
}

// Image Processing Loop
void ImageGrabber::processImages()
{
    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 0;
        // Check if there is any image in the buffer
        if (!img0Buf.empty())
        {
            {
                std::lock_guard<std::mutex> lock(mBufMutex);
                im = getImage(img0Buf.front());
                tIm = img0Buf.front()->header.stamp.sec + img0Buf.front()->header.stamp.nanosec * 1e-9;
                img0Buf.pop();
            }

            if (im.empty()) {
                continue;
            }

            if (mbClahe) {
                mClahe->apply(im, im);  // Apply CLAHE if enabled
            }

            // Process the image in the SLAM system
            Sophus::SE3f curr_pose;
            try {
                curr_pose = mpSLAM->TrackMonocular(im, tIm);
            } catch (const std::exception &e) {
                std::cerr << "Exception caught: " << e.what() << std::endl;
            }
            
            //publish pose
            publishSE3fToOdom(curr_pose);           
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// Publishing the Estimated Pose
void ImageGrabber::publishSE3fToOdom(const Sophus::SE3f& se3) {
    
    // Extract the translation (position)
    Eigen::Vector3f translation = se3.translation();
    odom_msg_.pose.pose.position.x = translation.x();
    odom_msg_.pose.pose.position.y = translation.y();
    odom_msg_.pose.pose.position.z = translation.z();

    // Extract the rotation and convert to quaternion
    Eigen::Matrix3f rotation_matrix = se3.rotationMatrix();
    Eigen::Quaternionf quaternion(rotation_matrix);

    odom_msg_.pose.pose.orientation.x = quaternion.x();
    odom_msg_.pose.pose.orientation.y = quaternion.y();
    odom_msg_.pose.pose.orientation.z = quaternion.z();
    odom_msg_.pose.pose.orientation.w = quaternion.w();


    odom_pub_->publish(odom_msg_);
}
