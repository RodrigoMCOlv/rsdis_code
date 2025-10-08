#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class TurtleControl {
public:
  TurtleControl(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    rng_(std::random_device{}()),
    coin_(0, 1) {
    pnh_.param("linear_speed", linear_speed_, 0.3);
    pnh_.param("angular_speed", angular_speed_, 0.6);
    pnh_.param("bot_side", bot_side_, 0.220);
    pnh_.param("safety_threshold_front", safety_threshold_front_, 1.4);
    pnh_.param("safety_threshold_side", safety_threshold_side_, 1.6);
    pnh_.param("forward_arc", forward_arc_deg_, 40.0);
    pnh_.param("left_arc", left_arc_deg_, 25.0);
    pnh_.param("right_arc", right_arc_deg_, 25.0);

    laser_sub_ = nh_.subscribe("scan", 10, &TurtleControl::laserCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    control_timer_ = nh_.createTimer(ros::Duration(0.1), &TurtleControl::controlLoop, this);

    turn_duration_ = ros::Duration((M_PI / 6.0) / std::abs(angular_speed_));
    turn_deadline_ = ros::Time::now();
  }

private:
  enum class State { Stopped, Forward, TurnRight, TurnLeft, TurnArround };

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) { latest_scan_ = msg; }

  static double wrapToPi(double x) {
    x = std::fmod(x + M_PI, 2.0 * M_PI);
    if (x <= 0) x += 2.0 * M_PI;
    return x - M_PI;
  }

  static bool inRange(double x, double a, double b) { return x >= a && x <= b; }

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
    const double thr = bot_side_ * (0.5 + threshold);
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
    const double thr = bot_side_ * (0.5 + threshold);
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
    const double thr = bot_side_ * (0.5 + threshold);
    return min_d < thr;
  }

  void controlLoop(const ros::TimerEvent &) {
    geometry_msgs::Twist cmd;
    const ros::Time now_time = ros::Time::now();

    switch (state_) {
      case State::Forward:
        if (obstacleAhead(safety_threshold_front_) ||
            obstacleLeft(safety_threshold_side_) ||
            obstacleRight(safety_threshold_side_)) {
          state_ = State::Stopped;
        }
        break;

      case State::Stopped:
        if (obstacleAhead(safety_threshold_front_)) {
          const int direction = coin_(rng_);
          if (direction == 1) {
            state_ = State::TurnRight;
            ROS_INFO("Wall AHEAD -> random says RIGHT");
          } else {
            state_ = State::TurnLeft;
            ROS_INFO("Wall AHEAD -> random says LEFT");
          }
          turn_deadline_ = now_time + turn_duration_ * 2.0;
        } else if (obstacleLeft(safety_threshold_side_) && obstacleRight(safety_threshold_side_)) {
          state_ = State::TurnArround;
          turn_deadline_ = now_time + ros::Duration(M_PI / std::abs(angular_speed_));
          ROS_INFO("In a Corner -> Turning around");
        } else if (obstacleLeft(safety_threshold_side_)) {
          state_ = State::TurnRight;
          turn_deadline_ = now_time + turn_duration_;
          ROS_INFO("Obstacle LEFT -> turning RIGHT");
        } else if (obstacleRight(safety_threshold_side_)) {
          state_ = State::TurnLeft;
          turn_deadline_ = now_time + turn_duration_;
          ROS_INFO("Obstacle RIGHT -> turning LEFT");
        } else {
          state_ = State::Forward;
        }
        break;

      case State::TurnRight:
      case State::TurnLeft:
      case State::TurnArround:
        if (now_time >= turn_deadline_) {
          state_ = State::Stopped;
        }
        break;
    }

    ROS_INFO("Currently on: %s", stateToString(state_));

    switch (state_) {
      case State::Forward:
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
        break;
      case State::TurnRight:
        cmd.linear.x = 0.0;
        cmd.angular.z = -angular_speed_;
        break;
      case State::TurnLeft:
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed_;
        break;
      case State::TurnArround:
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed_;
        break;
      case State::Stopped:
      default:
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        break;
    }

    cmd_pub_.publish(cmd);
  }

  static const char *stateToString(State s) {
    switch (s) {
      case State::Stopped:   return "Stopped";
      case State::Forward:   return "Forward";
      case State::TurnRight: return "TurnRight";
      case State::TurnLeft:  return "TurnLeft";
      case State::TurnArround: return "TurnArround";
    }
    return "Unknown";
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber laser_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer control_timer_;
  sensor_msgs::LaserScan::ConstPtr latest_scan_;

  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double bot_side_{0.0};
  double safety_threshold_front_{0.0};
  double safety_threshold_side_{0.0};
  double forward_arc_deg_{0.0};
  double left_arc_deg_{0.0};
  double right_arc_deg_{0.0};

  State state_{State::Stopped};
  ros::Time turn_deadline_;
  ros::Duration turn_duration_{0.0};

  std::mt19937 rng_;
  std::uniform_int_distribution<int> coin_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TurtleControl node(nh, pnh);
  ros::spin();
  return 0;
}
