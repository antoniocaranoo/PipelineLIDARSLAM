#include <memory>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>
#include <utility>
#include <atomic>

#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "scancontext/Scancontext/Scancontext.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "kiss_matcher/KISSMatcher.hpp"

using std::placeholders::_1;

class IraSlamNode : public rclcpp::Node
{
public:
  IraSlamNode()
    : Node("ira_slam"),
      sc_manager_(),
      has_previous_position_(false),
      need_to_compute_descriptor_(false),
      stop_processing_(false),
      current_index_(0)
  {
    RCLCPP_INFO(this->get_logger(), "ira_slam node started");

    this->declare_parameter<double>("min_displacement", 1.0);
    this->declare_parameter<std::string>("odom_topic", "/kiss/odometry");
    this->declare_parameter<std::string>("pointcloud_topic", "/pointcloud_topic");

    // matcher config params
    this->declare_parameter<double>("matcher_resolution", 0.2);
    this->declare_parameter<int>("max_match_points", 60000);
    this->declare_parameter<double>("voxel_leaf_size", 0.1);

    min_displacement = this->get_parameter("min_displacement").as_double();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();

    matcher_resolution_ = (float)this->get_parameter("matcher_resolution").as_double();
    max_match_points_ = this->get_parameter("max_match_points").as_int();
    voxel_leaf_size_ = (float)this->get_parameter("voxel_leaf_size").as_double();

    // SCManager params 
    this->declare_parameter<int>("num_rings", 20);
    this->declare_parameter<int>("num_sectors", 60);
    this->declare_parameter<double>("max_radius", 80.0);
    this->declare_parameter<double>("lidar_height", 2.0);
    this->declare_parameter<int>("num_exclude_recent", 50);
    this->declare_parameter<int>("num_candidates_from_tree", 10);
    this->declare_parameter<double>("search_ratio", 0.1);
    this->declare_parameter<double>("sc_dist_thres", 0.13);
    this->declare_parameter<int>("tree_making_period", 50);

    int num_rings = this->get_parameter("num_rings").as_int();
    int num_sectors = this->get_parameter("num_sectors").as_int();
    double max_radius = this->get_parameter("max_radius").as_double();
    double lidar_height = this->get_parameter("lidar_height").as_double();
    int num_exclude_recent = this->get_parameter("num_exclude_recent").as_int();
    int num_candidates_from_tree = this->get_parameter("num_candidates_from_tree").as_int();
    double search_ratio = this->get_parameter("search_ratio").as_double();
    double sc_dist_thres = this->get_parameter("sc_dist_thres").as_double();
    int tree_making_period = this->get_parameter("tree_making_period").as_int();

    sc_manager_ = std::make_unique<SCManager>(
      num_rings,
      num_sectors,
      max_radius,
      lidar_height,
      num_exclude_recent,
      num_candidates_from_tree,
      search_ratio,
      sc_dist_thres,
      tree_making_period
    );

    kiss_matcher::KISSMatcherConfig config(matcher_resolution_);
    matcher_ = std::make_unique<kiss_matcher::KISSMatcher>(config);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&IraSlamNode::pointcloudCallback, this, _1));

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&IraSlamNode::odometryCallback, this, _1));

    processing_thread_ = std::thread([this]() { this->processQueue(); });
  }

  ~IraSlamNode()
  {
    stop_processing_.store(true);
    queue_cv_.notify_all();   
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

private:
  // ROS subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // params
  double min_displacement;
  float matcher_resolution_;
  int max_match_points_;
  float voxel_leaf_size_;

  std::unique_ptr<SCManager> sc_manager_;
  geometry_msgs::msg::Point previous_position_;
  bool has_previous_position_;
  std::atomic<bool> need_to_compute_descriptor_;

  std::unique_ptr<kiss_matcher::KISSMatcher> matcher_;

  // Coda thread-safe
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> keyframe_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // Worker
  std::thread processing_thread_;
  std::atomic<bool> stop_processing_;

  int current_index_;

  static constexpr double kPI = 3.14159265358979323846;

  std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::vector<Eigen::Vector3f> vec;
    vec.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      vec.emplace_back(pt.x, pt.y, pt.z);
    }
    return vec;
  }

  std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    std::vector<Eigen::Vector3f> vec;
    vec.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      vec.emplace_back(pt.x, pt.y, pt.z);
    }
    return vec;
  }

  void printPoints(const std::vector<Eigen::Vector3f>& v, const std::string &name, size_t n = 5) {
    size_t m = std::min(n, v.size());
    for (size_t i = 0; i < m; ++i) {
      RCLCPP_INFO(this->get_logger(), "%s[%zu] = %.6f %.6f %.6f",
                  name.c_str(), i, v[i].x(), v[i].y(), v[i].z());
    }
  }


  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!need_to_compute_descriptor_.load(std::memory_order_relaxed)) return;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      keyframe_queue_.push(msg);
    }
    queue_cv_.notify_one();
    need_to_compute_descriptor_.store(false, std::memory_order_relaxed);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto& current_position = msg->pose.pose.position;

    if (!has_previous_position_) {
      previous_position_ = current_position;
      has_previous_position_ = true;
      return;
    }

    double dx = current_position.x - previous_position_.x;
    double dy = current_position.y - previous_position_.y;
    double dz = current_position.z - previous_position_.z;

    double displacement = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (displacement >= min_displacement) {
      need_to_compute_descriptor_.store(true, std::memory_order_relaxed);
      previous_position_ = current_position;
    }
  }

  void processQueue()
  {
    while (!stop_processing_.load()) {
      sensor_msgs::msg::PointCloud2::SharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return stop_processing_.load() || !keyframe_queue_.empty();
        });
        if (stop_processing_.load()) break;
        msg = keyframe_queue_.front();
        keyframe_queue_.pop();
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *cloud);

      // Pulizia punti NaN/Inf secondo KISS-Matcher
      cloud->is_dense = false;
      cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
                                        [](const pcl::PointXYZI &p){
                                            return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
                                        }),
                          cloud->points.end());

      double timestamp = rclcpp::Time(msg->header.stamp).seconds();
      sc_manager_->makeAndSaveScancontextAndKeys(*cloud, timestamp);

      auto [loop_index, yaw_diff_rad] = sc_manager_->detectLoopClosureID();

      int target_index = sc_manager_->getNumFrames() - 1;

      if (loop_index >= 0) {
        auto loop_cloud = sc_manager_->getPointCloud(loop_index);

        if (loop_cloud && !loop_cloud->points.empty() && !cloud->points.empty()) {

          // Convert to vector<Eigen::Vector3f>
          auto src_vec = convertCloudToVec(*loop_cloud);
          auto tgt_vec = convertCloudToVec(*cloud);

          RCLCPP_INFO(this->get_logger(), "KISSMatcher: source size = %zu, target size = %zu", src_vec.size(), tgt_vec.size());

          if (src_vec.size() < 10 || tgt_vec.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Too few points for matching (src=%zu tgt=%zu)", src_vec.size(), tgt_vec.size());
            need_to_compute_descriptor_.store(false, std::memory_order_relaxed);
            continue; 
          }

          kiss_matcher::RegistrationSolution sol;
          try {
            sol = matcher_->estimate(src_vec, tgt_vec); 
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in matcher_.estimate(): %s", e.what());
            matcher_->clear(); matcher_->reset(); matcher_->resetSolver();
            need_to_compute_descriptor_.store(false, std::memory_order_relaxed);
            continue; 
          }

          bool invalid = false;
          try {
            if (sol.rotation.size() == 0 || sol.translation.size() == 0) invalid = true;
          } catch (...) {
            invalid = true;
          }

          if (invalid) {
            RCLCPP_WARN(this->get_logger(), "KISSMatcher failed to compute a valid transform");
            matcher_->clear(); 
            matcher_->reset(); 
            matcher_->resetSolver();
            need_to_compute_descriptor_.store(false, std::memory_order_relaxed);
            continue; 
          }

          Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
          T.block<3,3>(0,0) = sol.rotation.cast<float>();
          T.block<3,1>(0,3) = sol.translation.cast<float>();

          RCLCPP_INFO(this->get_logger(), "[Loop found] btn %d and %d, yaw diff: %.2f deg.", loop_index, target_index, yaw_diff_rad * 180.0 / kPI);

          double src_time = sc_manager_->getTimestamp(loop_index);
          double trg_time = sc_manager_->getTimestamp(target_index);

          std::ofstream file("loop_closures.txt", std::ios::app);
          if (file.is_open()) {
              file << "SRC_TIME=" << src_time << " TRG_TIME=" << trg_time << "\n";
              file << T << "\n";
              file.close();
          }

          matcher_->clear(); 
          matcher_->reset();
          matcher_->resetSolver();
        }
      }
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IraSlamNode>());
  rclcpp::shutdown();
  return 0;
}
