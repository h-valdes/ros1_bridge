#include <string>

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"

int main(int argc, char* argv[]) {
    // ROS 1 node
    ros::init(argc, argv, "ros_bridge");
    ros::NodeHandle ros1_node;

    // ROS 2 node
    rclcpp::init(argc, argv);
    auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

    // bridge the /tag_detections topic
    std::string topic_name_tag_detections = "/tag_detections";
    std::string ros1_type_name_tag_detections = "apriltag_ros/AprilTagDetectionArray";
    std::string ros2_type_name_tag_detections = "apriltag_msgs/msg/AprilTagDetectionArray";
    size_t queue_size_tag_detections = 10;

    // bridge the /tf topic
    std::string topic_name_tf = "/tf";
    std::string ros1_type_name_tf = "tf2_msgs/TFMessage";
    std::string ros2_type_name_tf = "tf2_msgs/msg/TFMessage";
    size_t queue_size_tf = 10;

    auto handles_tag_detections = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_tag_detections, 
        ros2_type_name_tag_detections, topic_name_tag_detections, 
        queue_size_tag_detections);

    auto handles_tf = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_tf, 
        ros2_type_name_tf, topic_name_tf, 
        queue_size_tf);

    // ROS 1 asynchronous spinner
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    // ROS 2 spinning loop
    rclcpp::executors::SingleThreadedExecutor executor;
    while (ros1_node.ok() && rclcpp::ok()) {
        executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
    }

    return 0;
}
