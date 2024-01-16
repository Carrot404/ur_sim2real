/*
 * @Description: io_mimic_controller.hpp
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2024-01-06 09:05:12
 * @LastEditTime: 2024-01-06 09:53:47
 */

#ifndef IO_MIMIC_CONTROLLER_HPP_
#define IO_MIMIC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/srv/set_io.hpp"

namespace ur_sim2real
{
  class IOMimicController : public controller_interface::ControllerInterface
  {
  public:
    IOMimicController() = default;
    ~IOMimicController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_init() override;

  protected:
    // subscriber for io_states
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_states_subscriber_;
    // realtime buffer for io_states
    realtime_tools::RealtimeBuffer<std::shared_ptr<ur_msgs::msg::IOStates>> io_states_buffer_;
    std::shared_ptr<ur_msgs::msg::IOStates> io_states_msg_;

    bool current_io_state_ = false;

    // client for set_io service
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;
  }; // class IOMimicController

} // namespace ur_sim2real

#endif // IO_MIMIC_CONTROLLER_HPP_