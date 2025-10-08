
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <random>
#include <limits>
#include <algorithm>
#include <cmath>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Dense>

struct Circle {
  Eigen::Vector2d center;
  double radius{0.0};
  std::vector<int> inliers;
  bool valid{false};
};

class CircleDetectorNode : public rclcpp::Node {
public:
  CircleDetectorNode() : Node("circle_detector_node") {
    // Declare & get parameters
    target_radius_      = this->declare_parameter<double>("target_radius", 0.12);
    radius_tolerance_   = this->declare_parameter<double>("radius_tolerance", 0.1);
    min_inliers_        = this->declare_parameter<int>("min_inliers", 5);
    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.12);
    ransac_iterations_  = this->declare_parameter<int>("ransac_iterations", 3000);
    downsample_step_    = this->declare_parameter<int>("downsample_step", 1);
    min_range_          = this->declare_parameter<double>("min_range", 0.02);
    max_range_          = this->declare_parameter<double>("max_range", 2.0);
    frame_id_           = this->declare_parameter<std::string>("frame_id", "laser_frame");
    input_topic_        = this->declare_parameter<std::string>("scan_topic", "/scan");

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CircleDetectorNode::scanCallback, this, std::placeholders::_1));

    centers_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("circle_centers", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("circles", 10);

    rng_.seed(std::random_device{}());

    RCLCPP_INFO(get_logger(), "circle_detector_node started. Looking for radius ~ %.3f m", target_radius_);
  }

private:
  // Parameters
  double target_radius_;
  double radius_tolerance_;
  int    min_inliers_;
  double distance_threshold_;
  int    ransac_iterations_;
  int    downsample_step_;
  double min_range_, max_range_;
  std::string frame_id_;
  std::string input_topic_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  std::mt19937 rng_;

  static bool computeCircleFrom3(const Eigen::Vector2d &p1,
                                 const Eigen::Vector2d &p2,
                                 const Eigen::Vector2d &p3,
                                 Eigen::Vector2d &center, double &radius) {
    // Check collinearity via area (cross product)
    double area = std::abs(0.5 * ((p2 - p1).x() * (p3 - p1).y() - (p2 - p1).y() * (p3 - p1).x()));
    if (area < 1e-6) return false; // Nearly collinear

    // Circle from 3 points using perpendicular bisector intersection
    Eigen::Vector2d mid12 = 0.5 * (p1 + p2);
    Eigen::Vector2d mid23 = 0.5 * (p2 + p3);
    Eigen::Vector2d d12 = p2 - p1;
    Eigen::Vector2d d23 = p3 - p2;

    // Perpendicular directions
    Eigen::Vector2d n12(-d12.y(), d12.x());
    Eigen::Vector2d n23(-d23.y(), d23.x());

    // Solve: mid12 + t*n12 = mid23 + s*n23
    Eigen::Matrix2d A;
    A << n12.x(), -n23.x(),
         n12.y(), -n23.y();
    Eigen::Vector2d b = mid23 - mid12;

    if (std::abs(A.determinant()) < 1e-9) return false;

    Eigen::Vector2d ts = A.fullPivLu().solve(b);
    center = mid12 + ts(0) * n12;
    radius = (center - p1).norm();
    return std::isfinite(radius);
  }

  static Circle kasaRefit(const std::vector<Eigen::Vector2d> &pts, const std::vector<int> &inliers) {
    Circle out;
    if (inliers.size() < 3) return out;

    // Solve A*[a b c]^T = bvec, with A = [x y 1], bvec = -(x^2 + y^2)
    Eigen::MatrixXd A(inliers.size(), 3);
    Eigen::VectorXd b(inliers.size());
    for (size_t i = 0; i < inliers.size(); ++i) {
      const auto &p = pts[inliers[i]];
      A(i, 0) = p.x();
      A(i, 1) = p.y();
      A(i, 2) = 1.0;
      b(i) = -(p.x() * p.x() + p.y() * p.y());
    }
    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    double a = sol(0), c = sol(1), d = sol(2);
    Eigen::Vector2d center(-a * 0.5, -c * 0.5);
    double R_sq = center.squaredNorm() - d;
    if (R_sq <= 0.0 || !std::isfinite(R_sq)) return out;
    out.center = center;
    out.radius = std::sqrt(R_sq);
    out.inliers = inliers;
    out.valid = true;
    return out;
  }

  Circle ransacDetect(const std::vector<Eigen::Vector2d> &pts) {
    Circle best;
    if (pts.size() < 3) return best;

    std::uniform_int_distribution<int> dist(0, static_cast<int>(pts.size()) - 1);

    for (int it = 0; it < ransac_iterations_; ++it) {
      // Sample 3 unique indices
      int i1 = dist(rng_), i2 = dist(rng_), i3 = dist(rng_);
      if (i1 == i2 || i1 == i3 || i2 == i3) { --it; continue; }

      Eigen::Vector2d center;
      double radius;
      if (!computeCircleFrom3(pts[i1], pts[i2], pts[i3], center, radius)) continue;

      // Quick radius pre-check
      if (std::abs(radius - target_radius_) > (radius_tolerance_ + 0.05)) continue;

      // Score inliers
      std::vector<int> inliers;
      inliers.reserve(pts.size());
      for (int k = 0; k < static_cast<int>(pts.size()); ++k) {
        double err = std::abs((pts[k] - center).norm() - radius);
        if (err < distance_threshold_) inliers.push_back(k);
      }

      if (static_cast<int>(inliers.size()) > static_cast<int>(best.inliers.size())) {
        // Refit with Kåsa on inliers
        Circle refined = kasaRefit(pts, inliers);
        if (refined.valid &&
            std::abs(refined.radius - target_radius_) <= radius_tolerance_ &&
            static_cast<int>(refined.inliers.size()) >= min_inliers_) {
          best = refined;
        }
      }
    }

    return best;
  }

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // --- collect gated points (range filter only) ---
  std::vector<Eigen::Vector2d> all_pts;
  all_pts.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float r = msg->ranges[i];
    if (!std::isfinite(r) || r < static_cast<float>(min_range_) || r > static_cast<float>(max_range_)) continue;
    const double th = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    all_pts.emplace_back(r * std::cos(th), r * std::sin(th));
  }

  // --- cluster contiguous returns in angle order ---
  auto make_clusters = [&](const auto& ranges, double a_min, double a_inc)
      -> std::vector<std::vector<Eigen::Vector2d>> {
    const double jump_thresh = 0.12;     // meters between adjacent beams
    const int    min_sz = 10, max_sz = 75; // << your constraint

    std::vector<std::vector<Eigen::Vector2d>> clusters;
    std::vector<Eigen::Vector2d> cur;

    auto flush = [&](){
      if ((int)cur.size() >= min_sz && (int)cur.size() <= max_sz) clusters.push_back(cur);
      cur.clear();
    };

    float prev_r = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < ranges.size(); ++i) {
      const float r = ranges[i];
      if (!std::isfinite(r) || r < static_cast<float>(min_range_) || r > static_cast<float>(max_range_)) {
        if (!cur.empty()) flush();
        prev_r = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      const double th = a_min + static_cast<double>(i) * a_inc;
      const Eigen::Vector2d p(r * std::cos(th), r * std::sin(th));
      if (!std::isfinite(prev_r) || std::abs(r - prev_r) < jump_thresh) cur.push_back(p);
      else { flush(); cur.push_back(p); }
      prev_r = r;
    }
    if (!cur.empty()) flush();

    // (optional) light PCA filter to drop very straight segments
    std::vector<std::vector<Eigen::Vector2d>> curved_only;
    curved_only.reserve(clusters.size());
    for (auto &c : clusters) {
      Eigen::Vector2d mean = Eigen::Vector2d::Zero();
      for (auto &p : c) mean += p;
      mean /= (double)c.size();
      Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
      for (auto &p : c) { Eigen::Vector2d d=p-mean; C += d*d.transpose(); }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
      if (es.info()!=Eigen::Success) continue;
      const double lam0 = es.eigenvalues()(0), lam1 = es.eigenvalues()(1);
      if (lam1<=1e-9) continue;
      const double ratio = lam0/lam1;
      if (ratio > 0.05) curved_only.push_back(c);
    }
    return curved_only;
  };

  auto clusters = make_clusters(msg->ranges, msg->angle_min, msg->angle_increment);

  // --- force your inlier threshold and inlier count policy ---
  distance_threshold_ = 0.12;                      // equal to radius as requested
  const int required_inliers = std::max(10, min_inliers_);  // at least 10

  // --- RANSAC per cluster ---
  std::vector<Circle> valids;
  for (auto &c : clusters) {
    Circle cand = ransacDetect(c);
    if (cand.valid &&
        std::abs(cand.radius - target_radius_) <= radius_tolerance_ &&
        (int)cand.inliers.size() >= required_inliers) {
      valids.push_back(std::move(cand));
    }
  }

  // --- publish results ---
  geometry_msgs::msg::PoseArray poses;
  poses.header.stamp = msg->header.stamp;
  poses.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

  visualization_msgs::msg::MarkerArray marr;
  int id = 0;

  visualization_msgs::msg::Marker del;
  del.header = poses.header;
  del.ns = "circle_detector";
  del.id = id++;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marr.markers.push_back(del);

  if (!valids.empty()) {
    for (const auto &cir : valids) {
      geometry_msgs::msg::Pose p;
      p.position.x = cir.center.x();
      p.position.y = cir.center.y();
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      poses.poses.push_back(p);

      visualization_msgs::msg::Marker center_mk;
      center_mk.header = poses.header;
      center_mk.ns = "circle_detector";
      center_mk.id = id++;
      center_mk.type = visualization_msgs::msg::Marker::SPHERE;
      center_mk.action = visualization_msgs::msg::Marker::ADD;
      center_mk.pose = p;
      center_mk.scale.x = center_mk.scale.y = center_mk.scale.z = 0.05;
      center_mk.color.a = 1.0; center_mk.color.r = 0.0f; center_mk.color.g = 1.0f; center_mk.color.b = 0.0f;
      center_mk.lifetime = rclcpp::Duration(0, 500000000);
      marr.markers.push_back(center_mk);

      visualization_msgs::msg::Marker circle_mk;
      circle_mk.header = poses.header;
      circle_mk.ns = "circle_detector";
      circle_mk.id = id++;
      circle_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
      circle_mk.action = visualization_msgs::msg::Marker::ADD;
      circle_mk.pose.orientation.w = 1.0;
      circle_mk.scale.x = 0.01;
      circle_mk.color.a = 1.0; circle_mk.color.r = 0.0f; circle_mk.color.g = 0.5f; circle_mk.color.b = 1.0f;
      circle_mk.lifetime = rclcpp::Duration(0, 500000000);

      const int SEG = 72;
      circle_mk.points.reserve(SEG + 1);
      for (int i = 0; i <= SEG; ++i) {
        const double th = 2.0 * M_PI * (double)i / SEG;
        geometry_msgs::msg::Point pt;
        pt.x = cir.center.x() + cir.radius * std::cos(th);
        pt.y = cir.center.y() + cir.radius * std::sin(th);
        pt.z = 0.0;
        circle_mk.points.push_back(pt);
      }
      marr.markers.push_back(circle_mk);
    }
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
      "No valid circles (clusters kept with size 10..40: %zu, total gated pts=%zu)",
      clusters.size(), all_pts.size());
  }

  centers_pub_->publish(poses);
  markers_pub_->publish(marr);
}



};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
