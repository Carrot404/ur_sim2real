#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/srv/set_io.hpp"

class IOTest : public rclcpp::Node
{
  public:
    IOTest() : Node("io_test")
    {
      client_ = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
      while (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for the service to be up...");
      }
      RCLCPP_INFO(this->get_logger(), "Service is up!");

      auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
      request->fun = request->FUN_SET_DIGITAL_OUT;
      request->pin = request->PIN_DIGITAL_OUT_0;
      request->state = request->STATE_ON;
      auto future = client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "IO set!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IO");
      }
    }

    private:
      rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IOTest>());
  rclcpp::shutdown();
  return 0;
}
