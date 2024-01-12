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

  controller_interface::return_type JointMimicController::init(
      const std::string &controller_name)
  {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK)
    {
      return ret;
    }

    std::string tf_prefix_sim = "ursim.";
    std::string tf_prefix_real = "ur3e.";
    std::vector<std::string> joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    std::string joint_state_topic = "/URSIM/joint_states";

    try
    {
      auto_declare<std::string>("tf_prefix_sim", tf_prefix_sim);

      auto_declare<std::string>("tf_prefix_real", tf_prefix_real);

      auto_declare<std::vector<std::string>>("joints", joints);

      auto_declare<std::string>("joint_state_topic", joint_state_topic);
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::InterfaceConfiguration
  JointMimicController::command_interface_configuration() const
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

  controller_interface::InterfaceConfiguration
  JointMimicController::state_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  CallbackReturn JointMimicController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    std::vector<std::string> joint_names = node_->get_parameter("joints").as_string_array();
    if (joint_names.empty())
    {
      RCLCPP_ERROR(LOGGER, "'joints' parameter was empty");
      return CallbackReturn::ERROR;
    }

    num_joints_ = joint_names.size();
    joint_names_sim_.resize(num_joints_);
    joint_names_real_.resize(num_joints_);
    position_cmd_.resize(num_joints_);

    std::string tf_prefix_sim = node_->get_parameter("tf_prefix_sim").as_string();
    if (!tf_prefix_sim.empty())
    {
      for (size_t i = 0; i < num_joints_; ++i)
      {
        joint_names_sim_[i] = tf_prefix_sim + joint_names[i];
      }
    }

    std::string tf_prefix_real = node_->get_parameter("tf_prefix_real").as_string();
    if (!tf_prefix_real.empty())
    {
      for (size_t i = 0; i < num_joints_; ++i)
      {
        joint_names_real_[i] = tf_prefix_real + joint_names[i];
      }
    }

    std::string joint_state_topic = node_->get_parameter("joint_state_topic").as_string();
    if (joint_state_topic.empty())
    {
      RCLCPP_ERROR(LOGGER, "Joint state topic is empty!");
      return CallbackReturn::ERROR;
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
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  // Fill ordered_interfaces with references to the matching interfaces
  // in the same order as in joint_names
  template <typename T>
  bool get_ordered_interfaces(
      std::vector<T> &unordered_interfaces, const std::vector<std::string> &joint_names,
      const std::string &interface_type, std::vector<std::reference_wrapper<T>> &ordered_interfaces)
  {
    for (const auto &joint_name : joint_names)
    {
      for (auto &command_interface : unordered_interfaces)
      {
        if (
            (command_interface.get_name() == joint_name) &&
            (command_interface.get_interface_name() == interface_type))
        {
          ordered_interfaces.push_back(std::ref(command_interface));
        }
      }
    }

    return joint_names.size() == ordered_interfaces.size();
  }

  CallbackReturn JointMimicController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {

    if (!controller_interface::get_ordered_interfaces(command_interfaces_,
                                                      joint_names_real_,
                                                      "position",
                                                      position_command_interfaces_))
    {
      RCLCPP_ERROR(LOGGER, "Expected %zu command interfaces, got %zu.",
                   joint_names_real_.size(),
                   position_command_interfaces_.size());
      return CallbackReturn::ERROR;
    }

    // reset command buffer if a command came through callback when controller was inactive
    joint_state_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>(nullptr);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointMimicController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    position_command_interfaces_.clear();

    joint_state_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JointMimicController::update()
  {
    // read joint state from buffer
    joint_state_msg_ = *joint_state_buffer_.readFromRT();
    if (!joint_state_msg_.get())
    {
      RCLCPP_INFO(LOGGER, "No joint state message received");
      return controller_interface::return_type::OK;
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