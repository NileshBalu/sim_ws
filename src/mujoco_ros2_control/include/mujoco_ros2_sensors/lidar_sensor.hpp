#ifndef MUJOCO_ROS2_SENSORS_LIDAR_SENSOR_HPP
#define MUJOCO_ROS2_SENSORS_LIDAR_SENSOR_HPP

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>
#include <vector>
#include <atomic>

namespace mujoco_ros2_sensors
{

struct LidarSensorStruct
{
  std::string frame_id;
  std::string body_name;
  std::vector<int> sensor_ids;
  std::vector<int> range_sensor_adrs;
  double range_min;
  double range_max;
};

class LidarSensor
{
public:
  LidarSensor(
    rclcpp::Node::SharedPtr &node, mjModel * model, mjData * data,
    const LidarSensorStruct & sensor, std::atomic<bool>* stop, double frequency);

  void update();

private:
  rclcpp::Node::SharedPtr node_;
  mjModel* model_;
  mjData* data_;
  LidarSensorStruct sensor_;

  using LidarPublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::PointCloud2>;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<LidarPublisher> lidar_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters and Buffers for mj_multiRay
  double min_theta_, max_theta_, min_phi_, max_phi_;
  int h_samples_;
  int v_samples_;
  std::vector<mjtNum> ray_vecs_, ray_dists_, local_ray_vecs_;
  std::vector<int> ray_geomids_;
};

}  // namespace mujoco_ros2_sensors

#endif  // MUJOCO_ROS2_SENSORS_LIDAR_SENSOR_HPP