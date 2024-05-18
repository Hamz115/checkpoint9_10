#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach") {
    this->declare_parameter<double>("obstacle", 0.3);
    this->declare_parameter<int>("degrees", 90);

    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&PreApproach::move_forward, this));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    for (const auto & range : msg->ranges) {
      if (range < obstacle_) {
        stop_robot();
        start_rotation();
        break;
      }
    }
  }

  void move_forward() {
    if (!obstacle_detected_) {
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = 0.5;
      vel_pub_->publish(msg);
    }
  }

  void stop_robot() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    vel_pub_->publish(msg);
    obstacle_detected_ = true;
    RCLCPP_INFO(this->get_logger(), "Obstacle detected, stopping the robot.");
  }

  void start_rotation() {
    double target_angle = static_cast<double>(degrees_) * M_PI / 180.0; // Convert degrees to radians
    double current_angle = 0.0;
    auto rotation_start_time = this->now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && std::fabs(current_angle) < std::fabs(target_angle)) {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = (degrees_ > 0) ? 0.5 : -0.5; // Rotate with 0.5 rad/s
        vel_pub_->publish(msg);

        // Read the gyroscope sensor and update the current angle
        current_angle += (this->now() - rotation_start_time).seconds() * msg.angular.z;
        rotation_start_time = this->now();

        rate.sleep();
    }

    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Rotation complete.");
}

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double obstacle_;
  int degrees_;
  double current_angle;
  double target_angle;
  bool obstacle_detected_ = false;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}


// #include <chrono>
// #include <memory>
// #include <cmath>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// using namespace std::chrono_literals;

// class PreApproach : public rclcpp::Node {
// public:
//   PreApproach() : Node("pre_approach") {
//     this->declare_parameter<double>("obstacle", 1.0);
//     this->declare_parameter<int>("degrees", 90);

//     obstacle_ = this->get_parameter("obstacle").as_double();
//     degrees_ = this->get_parameter("degrees").as_int();

//     vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       "/scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10, std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(500ms, std::bind(&PreApproach::move_forward, this));
//   }

// private:
//   void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//     for (const auto & range : msg->ranges) {
//       if (range < obstacle_) {
//         stop_robot();
//         start_rotation();
//         break;
//       }
//     }
//   }

//   void move_forward() {
//     if (!obstacle_detected_) {
//       auto msg = geometry_msgs::msg::Twist();
//       msg.linear.x = 0.5;
//       vel_pub_->publish(msg);
//     }
//   }

//   void stop_robot() {
//     auto msg = geometry_msgs::msg::Twist();
//     msg.linear.x = 0.0;
//     vel_pub_->publish(msg);
//     obstacle_detected_ = true;
//     RCLCPP_INFO(this->get_logger(), "Obstacle detected, stopping the robot.");
//   }

//   void start_rotation() {
//     initial_yaw_ = current_yaw_;
//     target_yaw_ = normalize_angle(initial_yaw_ + degrees_ * M_PI / 180.0);

//     rclcpp::Rate rate(10);
//     while (rclcpp::ok()) {
//       double yaw_error = normalize_angle(target_yaw_ - current_yaw_);

//       if (std::fabs(yaw_error) < 0.01) {
//         stop_robot();
//         RCLCPP_INFO(this->get_logger(), "Rotation complete.");
//         return;
//       } else {
//         auto msg = geometry_msgs::msg::Twist();
//         msg.angular.z = (yaw_error > 0) ? 0.5 : -0.5;
//         vel_pub_->publish(msg);
//       }

//       rate.sleep();
//     }
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     current_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);
//   }

//   double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) {
//     double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
//     double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
//     return std::atan2(siny_cosp, cosy_cosp);
//   }

//   double normalize_angle(double angle) {
//     while (angle > M_PI) angle -= 2.0 * M_PI;
//     while (angle < -M_PI) angle += 2.0 * M_PI;
//     return angle;
//   }

//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   double obstacle_;
//   int degrees_;
//   bool obstacle_detected_ = false;
//   double initial_yaw_;
//   double current_yaw_;
//   double target_yaw_;
// };

// int main(int argc, char * argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PreApproach>());
//   rclcpp::shutdown();
//   return 0;
// }

