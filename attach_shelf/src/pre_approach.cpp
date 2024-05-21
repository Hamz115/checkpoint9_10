// #include <chrono>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>

// using namespace std::chrono_literals;

// class PreApproach : public rclcpp::Node {
// public:
//   PreApproach() : Node("pre_approach") {
//     this->declare_parameter<double>("obstacle", 0.3);
//     this->declare_parameter<int>("degrees", 90);

//     obstacle_ = this->get_parameter("obstacle").as_double();
//     degrees_ = this->get_parameter("degrees").as_int();

//     vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       "/scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

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
//     double target_angle = static_cast<double>(degrees_) * M_PI / 180.0; // Convert degrees to radians
//     double current_angle = 0.0;
//     auto rotation_start_time = this->now();
//     rclcpp::Rate rate(10);

//     while (rclcpp::ok() && std::fabs(current_angle) < std::fabs(target_angle)) {
//         auto msg = geometry_msgs::msg::Twist();
//         msg.angular.z = (degrees_ > 0) ? 0.5 : -0.5; // Rotate with 0.5 rad/s
//         vel_pub_->publish(msg);

//         // Read the gyroscope sensor and update the current angle
//         current_angle += (this->now() - rotation_start_time).seconds() * msg.angular.z;
//         rotation_start_time = this->now();

//         rate.sleep();
//     }

//     stop_robot();
//     RCLCPP_INFO(this->get_logger(), "Rotation complete.");
// }

//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   double obstacle_;
//   int degrees_;
//   double current_angle;
//   double target_angle;
//   bool obstacle_detected_ = false;
// };

// int main(int argc, char * argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PreApproach>());
//   rclcpp::shutdown();
//   return 0;
// }


