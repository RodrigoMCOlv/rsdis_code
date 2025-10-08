//TO COMPILE
//colcon build --symlink-install


#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node {
public:
  TurtleControl()
  : Node("turtle_control"),
    linear_speed_(declare_parameter("linear_speed", 0.3)),   // m/s
    angular_speed_(declare_parameter("angular_speed", 0.6)), // rad/s
    bot_side_(declare_parameter("bot_side", 0.220)),         // m
    safety_threshold_front_(declare_parameter("safety_threshold_front", 1.4)), // fraction of bot size
    safety_threshold_side_(declare_parameter("safety_threshold_side", 1.6)), // fraction of bot size

    forward_arc_deg_(declare_parameter("forward_arc", 40)),      // degrees
    left_arc_deg_(declare_parameter("left_arc", 25.0)),            // degrees
    right_arc_deg_(declare_parameter("right_arc", 25.0)),          // degrees
    rng_(std::random_device{}()),
    coin_(0, 1) // 0 or 1
  {
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&TurtleControl::laserCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
    control_timer_ = create_wall_timer(100ms, std::bind(&TurtleControl::controlLoop, this));

    // Correct 90-degree turn duration (π/2 rad)
    turn_duration_ = rclcpp::Duration::from_seconds((M_PI / 6) / std::abs(angular_speed_));
    turn_deadline_ = this->now();
  }

private:
  enum class State { Stopped, Forward, TurnRight, TurnLeft, TurnArround };

  // --- Laser callback ---
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; }

  // --- Helpers ---
  static inline double wrapToPi(double x) {
    x = std::fmod(x + M_PI, 2.0 * M_PI);
    if (x <= 0) x += 2.0 * M_PI;
    return x - M_PI;
  }

  static inline bool inRange(double x, double a, double b) { return x >= a && x <= b; }

  static inline double edgeThreshold(double bot_side, double edge_rad, double inflate) {
    const double safe_edge = std::min(edge_rad, M_PI / 2.0 - 1e-3);
    const double c = std::cos(safe_edge);
    return (bot_side / c) * (1.0 + inflate);
  }

  static const char* stateToString(State s) {
    switch (s) {
      case State::Stopped:   return "Stopped";
      case State::Forward:   return "Forward";
      case State::TurnRight: return "TurnRight";
      case State::TurnLeft:  return "TurnLeft";
      case State::TurnArround: return "TurnArround";    
    }
    return "Unknown";
  }

  // --- Obstacle checks ---
  bool obstacleAhead(double threshold) const {
    if (!latest_scan_) return false;
    const auto &s = *latest_scan_;
    const double half = (forward_arc_deg_ * M_PI / 180.0) / 2.0;

    double min_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < s.ranges.size(); ++i) {
      const float r = s.ranges[i];
      if (!std::isfinite(r)) continue;
      const double ang_i = s.angle_min + static_cast<double>(i) * s.angle_increment;
      const double delta = wrapToPi(ang_i);
      if (std::abs(delta) <= half) {
        min_d = std::min(min_d, static_cast<double>(r));
      }
    }
    //const double thr = edgeThreshold(bot_side_, half, threshold);
    const double thr = bot_side_ * (0.5 + threshold);       //half of robot size + threshold
    
    return min_d < thr;
  }

  bool obstacleLeft(double threshold) const {
    if (!latest_scan_) return false;
    const auto &s = *latest_scan_;
    const double half = (forward_arc_deg_ * M_PI / 180.0) / 2.0;
    const double left = (left_arc_deg_ * M_PI / 180.0);

    double min_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < s.ranges.size(); ++i) {
      const float r = s.ranges[i];
      if (!std::isfinite(r)) continue;
      const double ang_i = s.angle_min + static_cast<double>(i) * s.angle_increment;
      const double delta = wrapToPi(ang_i);
      if (inRange(delta, +half, +half + left)) {
        min_d = std::min(min_d, static_cast<double>(r));
      }
    }
    //const double thr = edgeThreshold(bot_side_, half + left, threshold);
    const double thr = bot_side_ * (0.5 + threshold);       //half of robot size + threshold
    
    return min_d < thr;
  }

  bool obstacleRight(double threshold) const {
    if (!latest_scan_) return false;
    const auto &s = *latest_scan_;
    const double half = (forward_arc_deg_ * M_PI / 180.0) / 2.0;
    const double right = (right_arc_deg_ * M_PI / 180.0);

    double min_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < s.ranges.size(); ++i) {
      const float r = s.ranges[i];
      if (!std::isfinite(r)) continue;
      const double ang_i = s.angle_min + static_cast<double>(i) * s.angle_increment;
      const double delta = wrapToPi(ang_i);
      if (inRange(delta, -(half + right), -half)) {
        min_d = std::min(min_d, static_cast<double>(r));
      }
    }
    //const double thr = edgeThreshold(bot_side_, half + right, threshold);
    const double thr = bot_side_ * (0.5 + threshold);       //half of robot size + threshold
    return min_d < thr;
  }

  // --- Control loop ---
  void controlLoop() {
    geometry_msgs::msg::Twist cmd; // default zeros
    const auto now_time = this->now();

    switch (state_) {
      case State::Forward:
        // Stop if ANY obstacle is too close
        if (obstacleAhead(safety_threshold_front_) ||
            obstacleLeft(safety_threshold_side_) ||
            obstacleRight(safety_threshold_side_)) {
          state_ = State::Stopped;
        }
        break;

      case State::Stopped:
        if (obstacleAhead(safety_threshold_front_)) {
          const int direction = coin_(rng_); // 0 or 1
          if (direction == 1) {
            state_ = State::TurnRight;   
            RCLCPP_INFO(this->get_logger(), "Wall AHEAD -> random says RIGHT");
          } else {
            state_ = State::TurnLeft;
            RCLCPP_INFO(this->get_logger(), "Wall AHEAD -> random says LEFT");
          }
          turn_deadline_ = now_time + turn_duration_ *2;
        }else if(obstacleLeft(safety_threshold_side_) && obstacleRight(safety_threshold_side_)){
          state_= State::TurnArround;
          turn_deadline_ = now_time + rclcpp::Duration::from_seconds(M_PI / std::abs(angular_speed_));
          RCLCPP_INFO(this->get_logger(), "In a Corner -> Turning arround");
        }else if (obstacleLeft(safety_threshold_side_)) {
          state_ = State::TurnRight;
          turn_deadline_ = now_time + turn_duration_;
          RCLCPP_INFO(this->get_logger(), "Obstacle LEFT -> turning RIGHT");
        }else if (obstacleRight(safety_threshold_side_)) {
          state_ = State::TurnLeft;
          turn_deadline_ = now_time + turn_duration_;
          RCLCPP_INFO(this->get_logger(), "Obstacle RIGHT -> turning LEFT");
        } else {
          state_ = State::Forward;
        }
        break;

      case State::TurnRight:
      case State::TurnLeft:
      case State::TurnArround:
        if (now_time >= turn_deadline_) {
          state_ = State::Stopped; // re-evaluate environment after timed turn
        }
        break;
    }

    // Safe logging: print a string, not using %s on an enum
    RCLCPP_INFO(this->get_logger(), "Currently on: %s", stateToString(state_));

    // Command for current state
    switch (state_) {
      case State::Forward:
        cmd.linear.x  = linear_speed_;
        cmd.angular.z = 0.0;
        break;
      case State::TurnRight:
        cmd.linear.x  = 0.0;
        cmd.angular.z = -angular_speed_;
        break;
      case State::TurnLeft:
        cmd.linear.x  = 0.0;
        cmd.angular.z =  angular_speed_;
        break;
      case State::Stopped:
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        break;
      case State::TurnArround:
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed_;
    }

    cmd_pub_->publish(cmd);
  }

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  // --- Parameters ---
  const double linear_speed_;
  const double angular_speed_;
  const double bot_side_;
  const double safety_threshold_front_;
  const double safety_threshold_side_;  
  const double forward_arc_deg_;
  const double left_arc_deg_;
  const double right_arc_deg_;

  // --- State ---
  State state_{State::Stopped};
  rclcpp::Time turn_deadline_;
  rclcpp::Duration turn_duration_{0, 0};

  // RNG
  std::mt19937 rng_;
  std::uniform_int_distribution<int> coin_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}



