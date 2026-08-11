#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TfOdometryRelay : public rclcpp::Node
{
public:
  TfOdometryRelay()
  : Node("tf_odometry_relay")
  {


    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/mecanum_base_controller/tf_odometry",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(
        &TfOdometryRelay::tf_callback,
        this,
        std::placeholders::_1));

    

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mecanum_base_controller/odometry",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(
        &TfOdometryRelay::odom_callback,
        this,
        std::placeholders::_1
      )
    );


    publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

    publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Relaying /mecanum_base_controller/tf_odometry -> /tf");
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    publisher_->publish(*msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    publisher_odom->publish(*msg);
  }



  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TfOdometryRelay>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
