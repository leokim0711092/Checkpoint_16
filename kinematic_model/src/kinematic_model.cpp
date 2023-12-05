#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

class VelPublisher : public rclcpp::Node
{
public:
  VelPublisher() : Node("Vel_publisher")
  {
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10, std::bind(&VelPublisher::wheelSpeedCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void wheelSpeedCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid wheel speed data size");
      return;
    }

    // wheel speeds in the order [front_left, front_right, rear_left, rear_right]
    double front_left = msg->data[0];
    double front_right = msg->data[1];
    double rear_left = msg->data[2];
    double rear_right = msg->data[3];

    geometry_msgs::msg::Twist vel;
    //transformation logic 
    vel.linear.x = r*(front_left + front_right)/2.0;
    vel.linear.y = r*(front_right - rear_left)/2.0;
    vel.angular.z = r*(front_right - rear_right)/(l+w);

    publisher_->publish(vel);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float r = 0.05;
  float l = 0.085;
  float w = 0.135;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
