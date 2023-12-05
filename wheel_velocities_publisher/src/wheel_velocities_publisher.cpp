#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("basic_motion");

  auto publisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);

  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Initialized wheel velocities publisher node");
  
  rclcpp::Rate r(1);
  std_msgs::msg::Float32MultiArray forward;
  forward.data = {1, 1, 1, 1};
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Move forward");
  for(int i = 0; i<3 ; i++) {
  publisher->publish(forward);
  r.sleep();
  }

  std_msgs::msg::Float32MultiArray backward;
  backward.data = {-1, -1, -1, -1};
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Move backward");
  for(int i = 0; i<3 ; i++) {
  publisher->publish(backward);
  r.sleep();
  }


  std_msgs::msg::Float32MultiArray left;
  left.data = {-1, 1, -1, 1};
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Move left");
  for(int i = 0; i<3 ; i++) {
  publisher->publish(left);
  r.sleep();
  }

  std_msgs::msg::Float32MultiArray right;
  right.data = {1, -1, 1, -1};
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Move right");
  for(int i = 0; i<3 ; i++) {
  publisher->publish(right);
  r.sleep();
  }

  std_msgs::msg::Float32MultiArray clockwise;
  clockwise.data = {1, -1, -1, 1};
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Turn clockwise");
  for(int i = 0; i<3 ; i++) {
  publisher->publish(clockwise);
  r.sleep();
  }  
  std_msgs::msg::Float32MultiArray counterclockwise;
  counterclockwise.data = {-1, 1, 1, -1};
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Turn counter-clockwise");
  for(int i = 0; i<3 ; i++) {
  publisher->publish(counterclockwise);
  r.sleep();
  }
  std_msgs::msg::Float32MultiArray stop;
  stop.data = {0, 0, 0, 0};
  publisher->publish(stop);
  RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "stop");

  rclcpp::shutdown();
  return 0;
}
