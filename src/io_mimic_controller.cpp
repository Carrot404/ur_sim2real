/*
 * @Description: io_mimic_controller.cpp
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2024-01-06 09:05:12
 * @LastEditTime: 2024-01-06 09:53:47
 */

#include "ur_sim2real/io_mimic_controller.hpp"

using namespace std::chrono_literals;

namespace
{
  const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_sim2real::IOMimicController");
}

namespace ur_sim2real
{
  controller_interface::CallbackReturn IOMimicController::on_init()
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration IOMimicController::command_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration IOMimicController::state_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn
  IOMimicController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    try
    {
      auto io_state_callback =
          [this](const std::shared_ptr<ur_msgs::msg::IOStates> msg)
      { io_states_buffer_.writeFromNonRT(msg); };
      io_states_subscriber_ = get_node()->create_subscription<ur_msgs::msg::IOStates>(
          "/URSIM/io_and_status_controller/io_states", rclcpp::SystemDefaultsQoS(), io_state_callback);
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during subscriber creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    set_io_client_ = get_node()->create_client<ur_msgs::srv::SetIO>("/UR3E/io_and_status_controller/set_io");
    // wait for server
    while (!set_io_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return controller_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  IOMimicController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  IOMimicController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type IOMimicController::update(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/)
  {
    // read joint state from buffer
    io_states_msg_ = *io_states_buffer_.readFromRT();
    if (!io_states_msg_.get())
    {
      RCLCPP_ERROR(LOGGER, "No IO state message received");
      return controller_interface::return_type::ERROR;
    }

    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = 1;
    request->pin = 0;

    if (io_states_msg_->digital_out_states[0].state != current_io_state_)
    {
      current_io_state_ = io_states_msg_->digital_out_states[0].state;
      if (current_io_state_)
      {
        request->state = 1;
      }
      else
      {
        request->state = 0;
      }
    }
    else
    {
      return controller_interface::return_type::OK;
    }

    // sync send request
    auto result = set_io_client_->async_send_request(request);

    return controller_interface::return_type::OK;
  }

} // namespace ur_sim2real

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_sim2real::IOMimicController, controller_interface::ControllerInterface)