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

    // bridge the is_ir Bool topic of /filtered_camera
    std::string topic_name_is_ir = "/filtered_camera/is_ir";
    std::string ros1_type_name_is_ir = "std_msgs/Bool";
    std::string ros2_type_name_is_ir = "std_msgs/msg/Bool";
    size_t queue_size_is_ir = 10;

    // bridge the camera_info topic of /filtered_camera
    std::string topic_name_camera_info = "/filtered_camera/camera_info";
    std::string ros1_type_name_camera_info = "sensor_msgs/CameraInfo";
    std::string ros2_type_name_camera_info = "sensor_msgs/msg/CameraInfo";
    size_t queue_size_camera_info = 10;

    // bridge the camera_info topic of /filtered_camera
    std::string topic_name_image = "/filtered_camera/image";
    std::string ros1_type_name_image = "sensor_msgs/Image";
    std::string ros2_type_name_image = "sensor_msgs/msg/Image";
    size_t queue_size_image = 10;

    // bridge the brightness_analyser/toggle_camera topic
    std::string topic_name_toggle = "/brightness_analyser/toggle_camera";
    std::string ros1_type_name_toggle = "std_msgs/Bool";
    std::string ros2_type_name_toggle = "std_msgs/msg/Bool";
    size_t queue_size_toggle = 10;

    // bridge the /tag_detections topic
    std::string topic_name_apriltag = "/tag_detections";
    std::string ros1_type_name_apriltag = "apriltag_ros/AprilTagDetectionArray";
    std::string ros2_type_name_apriltag = "apriltag_msgs/msg/AprilTagDetectionArray";
    size_t queue_size_apriltag = 10;

    auto handles_is_ir = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_is_ir, ros2_type_name_is_ir, topic_name_is_ir, queue_size_is_ir);

    auto handles_camera_info = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_camera_info, ros2_type_name_camera_info, topic_name_camera_info, queue_size_camera_info);

    auto handles_image = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_image, ros2_type_name_image, topic_name_image, queue_size_image);
    
    auto handles_toggle = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_toggle, ros2_type_name_toggle, topic_name_toggle, queue_size_toggle);

    auto handles_apriltag = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, ros1_type_name_apriltag, ros2_type_name_apriltag, topic_name_apriltag, queue_size_apriltag);

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
