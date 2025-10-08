#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <random>
#include <limits>
#include <algorithm>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Dense>

struct Circle {
  Eigen::Vector2d center;
  double radius{0.0};
  std::vector<int> inliers;
  bool valid{false};
};

class CircleDetectorNode {
public:
  CircleDetectorNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    pnh.param("target_radius", target_radius_, 0.12);
    pnh.param("radius_tolerance", radius_tolerance_, 0.1);
    pnh.param("min_inliers", min_inliers_, 5);
    pnh.param("distance_threshold", distance_threshold_, 0.12);
    pnh.param("ransac_iterations", ransac_iterations_, 3000);
    pnh.param("downsample_step", downsample_step_, 1);
    pnh.param("min_range", min_range_, 0.02);
    pnh.param("max_range", max_range_, 2.0);
    pnh.param<std::string>("frame_id", frame_id_, "laser_frame");
    pnh.param<std::string>("scan_topic", input_topic_, std::string("/scan"));

    scan_sub_ = nh.subscribe(input_topic_, 1, &CircleDetectorNode::scanCallback, this);
    centers_pub_ = nh.advertise<geometry_msgs::PoseArray>("circle_centers", 10);
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("circles", 10);

    rng_.seed(std::random_device{}());

    ROS_INFO("circle_detector_node started. Looking for radius ~ %.3f m", target_radius_);
  }

private:
  double target_radius_{0.12};
  double radius_tolerance_{0.1};
  int min_inliers_{5};
  double distance_threshold_{0.12};
  int ransac_iterations_{3000};
  int downsample_step_{1};
  double min_range_{0.02};
  double max_range_{2.0};
  std::string frame_id_{"laser_frame"};
  std::string input_topic_{"/scan"};

  ros::Subscriber scan_sub_;
  ros::Publisher centers_pub_;
  ros::Publisher markers_pub_;

  std::mt19937 rng_;

  static bool computeCircleFrom3(const Eigen::Vector2d &p1,
                                 const Eigen::Vector2d &p2,
                                 const Eigen::Vector2d &p3,
                                 Eigen::Vector2d &center, double &radius) {
    const double area = std::abs(0.5 * ((p2 - p1).x() * (p3 - p1).y() - (p2 - p1).y() * (p3 - p1).x()));
    if (area < 1e-6) return false;

    const Eigen::Vector2d mid12 = 0.5 * (p1 + p2);
    const Eigen::Vector2d mid23 = 0.5 * (p2 + p3);
    const Eigen::Vector2d d12 = p2 - p1;
    const Eigen::Vector2d d23 = p3 - p2;

    const Eigen::Vector2d n12(-d12.y(), d12.x());
    const Eigen::Vector2d n23(-d23.y(), d23.x());

    Eigen::Matrix2d A;
    A << n12.x(), -n23.x(),
         n12.y(), -n23.y();
    const Eigen::Vector2d b = mid23 - mid12;

    if (std::abs(A.determinant()) < 1e-9) return false;

    const Eigen::Vector2d ts = A.fullPivLu().solve(b);
    center = mid12 + ts(0) * n12;
    radius = (center - p1).norm();
    return std::isfinite(radius);
  }

  static Circle kasaRefit(const std::vector<Eigen::Vector2d> &pts, const std::vector<int> &inliers) {
    Circle out;
    if (inliers.size() < 3) return out;

    Eigen::MatrixXd A_mat(inliers.size(), 3);
    Eigen::VectorXd b_vec(inliers.size());
    for (size_t i = 0; i < inliers.size(); ++i) {
      const auto &p = pts[inliers[i]];
      A_mat(static_cast<int>(i), 0) = p.x();
      A_mat(static_cast<int>(i), 1) = p.y();
      A_mat(static_cast<int>(i), 2) = 1.0;
      b_vec(static_cast<int>(i)) = -(p.x() * p.x() + p.y() * p.y());
    }
    const Eigen::Vector3d sol = A_mat.colPivHouseholderQr().solve(b_vec);
    const double a = sol(0);
    const double c = sol(1);
    const double d = sol(2);
    const Eigen::Vector2d center(-a * 0.5, -c * 0.5);
    const double R_sq = center.squaredNorm() - d;
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
      const int i1 = dist(rng_);
      const int i2 = dist(rng_);
      const int i3 = dist(rng_);
      if (i1 == i2 || i1 == i3 || i2 == i3) {
        --it;
        continue;
      }

      Eigen::Vector2d center;
      double radius;
      if (!computeCircleFrom3(pts[i1], pts[i2], pts[i3], center, radius)) continue;

      if (std::abs(radius - target_radius_) > (radius_tolerance_ + 0.05)) continue;

      std::vector<int> inliers;
      inliers.reserve(pts.size());
      for (int k = 0; k < static_cast<int>(pts.size()); ++k) {
        const double err = std::abs((pts[k] - center).norm() - radius);
        if (err < distance_threshold_) inliers.push_back(k);
      }

      if (static_cast<int>(inliers.size()) > static_cast<int>(best.inliers.size())) {
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

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    std::vector<Eigen::Vector2d> all_pts;
    all_pts.reserve(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < static_cast<float>(min_range_) || r > static_cast<float>(max_range_)) continue;
      const double th = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      all_pts.emplace_back(r * std::cos(th), r * std::sin(th));
    }

    auto make_clusters = [&](const auto &ranges, double a_min, double a_inc)
        -> std::vector<std::vector<Eigen::Vector2d>> {
      const double jump_thresh = 0.12;
      const int min_sz = 10;
      const int max_sz = 75;

      std::vector<std::vector<Eigen::Vector2d>> clusters;
      std::vector<Eigen::Vector2d> cur;

      auto flush = [&]() {
        if (static_cast<int>(cur.size()) >= min_sz && static_cast<int>(cur.size()) <= max_sz) {
          clusters.push_back(cur);
        }
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
        if (!std::isfinite(prev_r) || std::abs(r - prev_r) < jump_thresh) {
          cur.push_back(p);
        } else {
          flush();
          cur.push_back(p);
        }
        prev_r = r;
      }
      if (!cur.empty()) flush();

      std::vector<std::vector<Eigen::Vector2d>> curved_only;
      curved_only.reserve(clusters.size());
      for (auto &c : clusters) {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (auto &p : c) mean += p;
        mean /= static_cast<double>(c.size());

        Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
        for (auto &p : c) {
          const Eigen::Vector2d d = p - mean;
          C += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
        if (es.info() != Eigen::Success) continue;
        const double lam0 = es.eigenvalues()(0);
        const double lam1 = es.eigenvalues()(1);
        if (lam1 <= 1e-9) continue;
        const double ratio = lam0 / lam1;
        if (ratio > 0.05) curved_only.push_back(c);
      }
      return curved_only;
    };

    auto clusters = make_clusters(msg->ranges, msg->angle_min, msg->angle_increment);

    distance_threshold_ = 0.12;
    const int required_inliers = std::max(10, min_inliers_);

    std::vector<Circle> valids;
    for (auto &c : clusters) {
      Circle cand = ransacDetect(c);
      if (cand.valid &&
          std::abs(cand.radius - target_radius_) <= radius_tolerance_ &&
          static_cast<int>(cand.inliers.size()) >= required_inliers) {
        valids.push_back(std::move(cand));
      }
    }

    geometry_msgs::PoseArray poses;
    poses.header.stamp = msg->header.stamp;
    poses.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

    visualization_msgs::MarkerArray marr;
    int id = 0;

    visualization_msgs::Marker del;
    del.header = poses.header;
    del.ns = "circle_detector";
    del.id = id++;
    del.action = visualization_msgs::Marker::DELETEALL;
    marr.markers.push_back(del);

    if (!valids.empty()) {
      for (const auto &cir : valids) {
        geometry_msgs::Pose p;
        p.position.x = cir.center.x();
        p.position.y = cir.center.y();
        p.position.z = 0.0;
        p.orientation.w = 1.0;
        poses.poses.push_back(p);

        visualization_msgs::Marker center_mk;
        center_mk.header = poses.header;
        center_mk.ns = "circle_detector";
        center_mk.id = id++;
        center_mk.type = visualization_msgs::Marker::SPHERE;
        center_mk.action = visualization_msgs::Marker::ADD;
        center_mk.pose = p;
        center_mk.scale.x = center_mk.scale.y = center_mk.scale.z = 0.05;
        center_mk.color.a = 1.0;
        center_mk.color.r = 0.0f;
        center_mk.color.g = 1.0f;
        center_mk.color.b = 0.0f;
        center_mk.lifetime = ros::Duration(0.5);
        marr.markers.push_back(center_mk);

        visualization_msgs::Marker circle_mk;
        circle_mk.header = poses.header;
        circle_mk.ns = "circle_detector";
        circle_mk.id = id++;
        circle_mk.type = visualization_msgs::Marker::LINE_STRIP;
        circle_mk.action = visualization_msgs::Marker::ADD;
        circle_mk.pose.orientation.w = 1.0;
        circle_mk.scale.x = 0.01;
        circle_mk.color.a = 1.0;
        circle_mk.color.r = 0.0f;
        circle_mk.color.g = 0.5f;
        circle_mk.color.b = 1.0f;
        circle_mk.lifetime = ros::Duration(0.5);

        const int SEG = 72;
        circle_mk.points.reserve(SEG + 1);
        for (int i = 0; i <= SEG; ++i) {
          const double th = 2.0 * M_PI * static_cast<double>(i) / SEG;
          geometry_msgs::Point pt;
          pt.x = cir.center.x() + cir.radius * std::cos(th);
          pt.y = cir.center.y() + cir.radius * std::sin(th);
          pt.z = 0.0;
          circle_mk.points.push_back(pt);
        }
        marr.markers.push_back(circle_mk);
      }
    } else {
      ROS_INFO_THROTTLE(1.0,
                        "No valid circles (clusters kept with size 10..40: %zu, total gated pts=%zu)",
                        clusters.size(), all_pts.size());
    }

    centers_pub_.publish(poses);
    markers_pub_.publish(marr);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "circle_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CircleDetectorNode node(nh, pnh);
  ros::spin();
  return 0;
}
