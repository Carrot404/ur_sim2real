/*
 * @Description: joint_mimic_controller.hpp
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2024-01-06 09:05:12
 * @LastEditTime: 2024-01-06 09:53:47
 */

#ifndef JOINT_MIMIC_CONTROLLER_HPP_
#define JOINT_MIMIC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "joint_mimic_controller_parameters.hpp"

namespace ur_sim2real
{
  class JointMimicController : public controller_interface::ControllerInterface
  {
  public:
    JointMimicController() = default;
    ~JointMimicController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_init() override;

  protected:
    // Parameters from ROS for JointMimicController
    std::shared_ptr<joint_mimic_controller::ParamListener> param_listener_;
    joint_mimic_controller::Params params_;

    size_t num_joints_;
    std::vector<std::string> joint_names_sim_;
    std::vector<std::string> joint_names_real_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_command_interfaces_;
    std::vector<double> position_cmd_;

    // subscriber for joint states of the simulated robot
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // realtime buffers for joint state messages
    realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> joint_state_buffer_;
    std::shared_ptr<sensor_msgs::msg::JointState> joint_state_msg_;

  }; // class JointMimicController

} // namespace ur_sim2real

#endif // JOINT_MIMIC_CONTROLLER_HPP_