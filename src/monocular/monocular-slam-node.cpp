#include "monocular-slam-node.hpp"
#include "rclcpp/rclcpp.hpp"

#include<opencv2/core/core.hpp>
#include<opencv2/core/mat.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Geometry>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    // Subscriber
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    // Publish SLAM Pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("slam/pose", 10);

}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;

    Sophus::SE3f s;

    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    // Convert SE3f = SE3<float> to Pose, position and orientation
    // Position
    geometry_msgs::msg::PoseStamped pose;
    Eigen::Vector3f translation = s.translation();
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();

    // Quaternion
    Eigen::Quaternionf quaternion = s.unit_quaternion();
    pose.pose.orientation.w = quaternion.w();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();

    RCLCPP_INFO(rclcpp::get_logger("Pose x"), "%f", translation.y());

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    // auto message = geometry_msgs::msg::Pose();
    // message.data = pose;
    // publisher_->publish(pose); // Publisher not working
    // TODO - Need to create a specific publisher to pose
    // The convertion from Sophus to pose is working. 
}
