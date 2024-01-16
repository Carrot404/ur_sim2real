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
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/srv/set_io.hpp"

using namespace std::chrono_literals;

class IO_Mimic_Node : public rclcpp::Node
{
public:
  IO_Mimic_Node() : Node("io_mimic_node")
  {
    // service client
    set_io_client_ = this->create_client<ur_msgs::srv::SetIO>("/UR3E/io_and_status_controller/set_io");
    // wait for server
    while (!set_io_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // subscriber
    io_states_subscriber_ = this->create_subscription<ur_msgs::msg::IOStates>(
        "/URSIM/io_and_status_controller/io_states", 10,
        [this](const ur_msgs::msg::IOStates::SharedPtr msg)
        {
          auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
          request->fun = 1;
          request->pin = 0;

          if (msg->digital_out_states[0].state != current_io_state_)
          {
            current_io_state_ = msg->digital_out_states[0].state;
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
            return;
          }

          // if (msg->digital_out_states[0].state == 1)
          // {
          //   request->state = 1;
          // }
          // else
          // {
          //   request->state = 0;
          // }

          // sync send request
          auto result = set_io_client_->async_send_request(request);
        });
  }

protected:
  // service client
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;

  // subscriber
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_states_subscriber_;

  bool current_io_state_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IO_Mimic_Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
