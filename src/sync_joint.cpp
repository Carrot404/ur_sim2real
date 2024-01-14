/*
 * @Description: sync_joint.cpp
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2024-01-06 09:05:12
 * @LastEditTime: 2024-01-06 09:53:47
 */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// create a ros2 node to synchronize the joint state from real robot to simulated robot
// real robot publishes the joint state to topic "/UR3E/joint_states"
// This node subscribes to topic "UR3E/joint_states" of real robot and publish the joint state to topic "/URSIM/joint_trajectory_controller/joint_trajectory" of simulated robot
// topic "/URSIM/joint_trajectory_controller/joint_trajectory" is "trajectory_msgs/JointTrajectory" type
// only publish once is fine when initializing the simulated robot

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sync_joint");

    // realtime buffers for joint state messages
    auto joint_state_buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>();

    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

    auto joint_state_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/URSIM/joint_trajectory_controller/joint_trajectory", 10);

    auto joint_state_callback =
        [&joint_state_buffer](const sensor_msgs::msg::JointState::SharedPtr msg)
    { joint_state_buffer.writeFromNonRT(msg); };

    auto joint_state_subscriber = node->create_subscription<sensor_msgs::msg::JointState>(
        "/UR3E/joint_states", rclcpp::SystemDefaultsQoS(), joint_state_callback);

    // while joint_state_msg is empty, still wait for the joint state from real robot
    // once joint_state_msg is not empty, publish the joint state to simulated robot

    while (rclcpp::ok())
    {
        // read joint state from buffer
        joint_state_msg = *joint_state_buffer.readFromRT();

        if (!joint_state_msg.get())
        {
            continue;
        }

        // publish joint state to joint trajectory controller
        auto joint_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

        joint_trajectory_msg->header.stamp = joint_state_msg->header.stamp;
        joint_trajectory_msg->header.frame_id = "base_link";
        joint_trajectory_msg->joint_names = joint_state_msg->name;
        joint_trajectory_msg->points.resize(1);
        joint_trajectory_msg->points[0].positions = joint_state_msg->position;
        joint_trajectory_msg->points[0].velocities = joint_state_msg->velocity;
        joint_trajectory_msg->points[0].accelerations = joint_state_msg->effort;
        joint_trajectory_msg->points[0].time_from_start = rclcpp::Duration(0, 0);
        
        joint_state_publisher->publish(*joint_trajectory_msg);
        rclcpp::spin_some(node);

        // wait for 2 seconds
        rclcpp::sleep_for(std::chrono::seconds(2));
        break;
    }

    rclcpp::shutdown();
    return 0;
}
