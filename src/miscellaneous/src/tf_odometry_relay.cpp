#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TfOdometryRelay : public rclcpp::Node
{
public:
  TfOdometryRelay()
  : Node("tf_odometry_relay")
  {
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/mecanum_base_controller/tf_odometry",
      rclcpp::QoS(100),
      std::bind(
        &TfOdometryRelay::tf_callback,
        this,
        std::placeholders::_1));

    publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf",
      rclcpp::QoS(100));

    RCLCPP_INFO(
      this->get_logger(),
      "Relaying /mecanum_base_controller/tf_odometry -> /tf");
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    publisher_->publish(*msg);
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TfOdometryRelay>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
