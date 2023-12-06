#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <thread>





class EightTraj : public rclcpp::Node
{
public:
  EightTraj() : Node("eight_trajectory")
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10, std::bind(&EightTraj::odomCallback, this, std::placeholders::_1), subscription_options);

    std::thread([this]() { execute(); }).detach();


  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::vector<std::vector<float>> motions_;
  nav_msgs::msg::Odometry odom_;
  bool pass;
  std_msgs::msg::Float32MultiArray stop;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void execute(){

    rclcpp::Rate r(1);
    motions_ = {{0, 1, -1}, {0.0, 1, 1}, {0.0, 1, 1}, {1.5708, 1, -1},{-3.1415, -1, -1}, {0.0, -1, 1}, {0.0, -1, 1}, {0.0, -1, -1} };
    int count = 0;
    for (const auto& motion : motions_)
    {
      float dx = motion[1];
      float dy = motion[2];
      float dphi = motion[0];
      RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Point %i" , ++count);
      for (int i = 0; i < 3; i++)
      {
        r.sleep();
      }

      for (int i = 0; i < 3; i++)
      {
        auto twist = velocity2twist(dphi, dx, dy);
        auto u = twist2wheels(twist[0], twist[1], twist[2]);
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = u;
        pub_->publish(msg);

        r.sleep();
      }
    }

    stop.data = {0, 0, 0, 0};
    pub_->publish(stop);
  
  } 

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
      odom_ = *msg;
      float phi = euler_degree_transform(odom_);

      RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Odom_Phi %f" , phi);
  }

    std::vector<float> velocity2twist(float dphi, float dx, float dy)
    {
        float phi = euler_degree_transform(odom_);
        RCLCPP_INFO(rclcpp::get_logger("wheel_velocities_publisher"), "Phi %f" , phi);

        Eigen::MatrixXd R(3, 3);
        R << 1, 0, 0,
            0, cos(phi), sin(phi),
            0, -sin(phi), cos(phi);

        Eigen::VectorXd v(3);
        v << dphi, dx, dy;

        Eigen::VectorXd twist = R * v;

        return {static_cast<float>(twist(0)), static_cast<float>(twist(1)), static_cast<float>(twist(2))};
    }

    std::vector<float> twist2wheels(double wz, double vx, double vy)
    {
        float r = 0.05;
        float l = 0.085;
        float w = 0.135;

        Eigen::MatrixXd H(4, 3);
        H << -l - w, 1, -1,
            l + w, 1, 1,
            l + w, 1, -1,
            -l - w, 1, 1;
        H /= r;

        Eigen::VectorXd twist(3);
        twist << wz, vx, vy;

        Eigen::VectorXd u = H * twist;

        return {static_cast<float>(u(0)), static_cast<float>(u(1)), static_cast<float>(u(2)), static_cast<float>(u(3))};
    }

    float euler_degree_transform(nav_msgs::msg::Odometry msg){
            float x = msg.pose.pose.orientation.x;
            float y = msg.pose.pose.orientation.y;
            float z = msg.pose.pose.orientation.z;
            float w = msg.pose.pose.orientation.w; 

        return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
    }


};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<EightTraj>();
  executor.add_node(node);
  executor.spin();


  rclcpp::shutdown();

  return 0;
}
