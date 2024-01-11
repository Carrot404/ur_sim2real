/*
 * @Description: joint_mimic_controller.cpp
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2024-01-06 09:05:12
 * @LastEditTime: 2024-01-06 09:53:47
 */

#include "ur_sim2real/joint_mimic_controller.hpp"

namespace
{
  const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_sim2real::JointMimicController");
}

namespace ur_sim2real
{
  controller_interface::CallbackReturn JointMimicController::on_init()
  {
    try
    {
      // Create the parameter listener and get the parameters
      param_listener_ = std::make_shared<joint_mimic_controller::ParamListener>(get_node());
      params_ = param_listener_->get_params();

      RCLCPP_INFO(LOGGER, "Loading JointMimicController!");
    }
    catch (std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration JointMimicController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.reserve(num_joints_);
    for (const auto &joint_name : joint_names_real_)
    {
      config.names.emplace_back(joint_name + std::string("/position"));
    }

    return config;
  }

  controller_interface::InterfaceConfiguration JointMimicController::state_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn
  JointMimicController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
    }
    std::vector<std::string> joint_names = params_.joints;
    if (joint_names.empty())
    {
      RCLCPP_ERROR(LOGGER, "No joint names specified in ROS2 parameters");
      return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names.size();
    joint_names_sim_.resize(num_joints_);
    joint_names_real_.resize(num_joints_);
    position_cmd_.resize(num_joints_);

    if (!params_.tf_prefix_sim.empty())
    {
      for (size_t i = 0; i < num_joints_; ++i)
      {
        joint_names_sim_[i] = params_.tf_prefix_sim + "/" + joint_names[i];
      }
    }
    if (!params_.tf_prefix_real.empty())
    {
      for (size_t i = 0; i < num_joints_; ++i)
      {
        joint_names_real_[i] = params_.tf_prefix_real + "/" + joint_names[i];
      }
    }

    if (params_.joint_state_topic.empty())
    {
      RCLCPP_ERROR(LOGGER, "Joint state topic is empty!");
      return controller_interface::CallbackReturn::ERROR;
    }

    try
    {
      auto joint_state_callback =
          [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg)
      { joint_state_buffer_.writeFromNonRT(msg); };
      joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
          params_.joint_state_topic, rclcpp::SystemDefaultsQoS(), joint_state_callback);
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during subscriber creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  JointMimicController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {

    if (!controller_interface::get_ordered_interfaces(command_interfaces_,
                                                      joint_names_real_,
                                                      "position",
                                                      position_command_interfaces_))
    {
      RCLCPP_ERROR(LOGGER, "Expected %zu command interfaces, got %zu.",
                   joint_names_real_.size(),
                   position_command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  JointMimicController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    position_command_interfaces_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JointMimicController::update(const rclcpp::Time & /*time*/,
                                                                 const rclcpp::Duration & /*period*/)
  {
    // read joint state from buffer
    joint_state_msg_ = *joint_state_buffer_.readFromRT();
    if (!joint_state_msg_.get())
    {
      RCLCPP_ERROR(LOGGER, "No joint state message received");
      return controller_interface::return_type::ERROR;
    }

    // write joint state to command interfaces
    for (size_t i = 0; i < num_joints_; ++i)
    {
      const auto &joint_name = joint_names_sim_[i];
      const auto &joint_state = joint_state_msg_->name;
      const auto &joint_position = joint_state_msg_->position;

      // find the index of the joint in the joint state message
      auto it = std::find(joint_state.begin(), joint_state.end(), joint_name);
      if (it == joint_state.end())
      {
        RCLCPP_ERROR(LOGGER, "Joint %s not found in joint state message", joint_name.c_str());
        return controller_interface::return_type::ERROR;
      }
      const auto joint_index = std::distance(joint_state.begin(), it);

      // write joint state to command interfaces
      position_cmd_[i] = joint_position[joint_index];
      position_command_interfaces_[i].get().set_value(position_cmd_[i]);
    }
    
    return controller_interface::return_type::OK;
  }

} // namespace ur_sim2real

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_sim2real::JointMimicController, controller_interface::ControllerInterface)