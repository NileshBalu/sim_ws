#include "mujoco_ros2_sensors/lidar_sensor.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>
#include <limits>

namespace mujoco_ros2_sensors
{

LidarSensor::LidarSensor(
  rclcpp::Node::SharedPtr &node, mjModel *model, mjData *data,
  const LidarSensorStruct &sensor, std::atomic<bool>* stop, double frequency)
: node_(node), model_(model), data_(data), sensor_(sensor)
{
    // Initialize publisher
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lidar", rclcpp::SystemDefaultsQoS());
    lidar_publisher_ = std::make_unique<LidarPublisher>(publisher_);

    // Read Lidar parameters
    min_angle_ = node_->declare_parameter<double>("min_angle", -M_PI);
    max_angle_ = node_->declare_parameter<double>("max_angle", M_PI);
    samples_ = node_->declare_parameter<int>("samples", 360);

    // Allow overriding range limits from ROS params, otherwise use defaults from struct
    if (node_->has_parameter("range_min")) {
        sensor_.range_min = node_->get_parameter("range_min").as_double();
    }
    if (node_->has_parameter("range_max")) {
        sensor_.range_max = node_->get_parameter("range_max").as_double();
    }

    // Resize buffers for mj_multiRay
    ray_vecs_.resize(samples_ * 3);
    local_ray_vecs_.resize(samples_ * 3);
    ray_dists_.resize(samples_);
    ray_geomids_.resize(samples_);

    // Precompute local vectors
    double angle_step = (samples_ > 1) ? (max_angle_ - min_angle_) / (samples_ - 1) : 0.0;
    for (int i = 0; i < samples_; ++i) {
        double angle = min_angle_ + i * angle_step;
        local_ray_vecs_[3 * i + 0] = std::cos(angle);
        local_ray_vecs_[3 * i + 1] = std::sin(angle);
        local_ray_vecs_[3 * i + 2] = 0.0;
    }

    RCLCPP_INFO(node_->get_logger(), "Lidar initialized on frame '%s' with %d samples. Range: [%.2f, %.2f]",
                sensor_.frame_id.c_str(), samples_, sensor_.range_min, sensor_.range_max);

    timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0 / frequency),
            std::bind(&LidarSensor::update, this));
}

void LidarSensor::update()
{
    if (lidar_publisher_->trylock()) {
        // 1. Get Sensor Pose
        mjtNum pos[3];
        mjtNum mat[9];
        int body_exclude = -1;

        if (!sensor_.body_name.empty()) {
            int body_id = mj_name2id(model_, mjOBJ_BODY, sensor_.body_name.c_str());
            if (body_id != -1) {
                mju_copy3(pos, data_->xpos + 3 * body_id);
                mju_copy(mat, data_->xmat + 9 * body_id, 9);
                body_exclude = body_id;
            } else {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Lidar body '%s' not found", sensor_.body_name.c_str());
                lidar_publisher_->unlock();
                return;
            }
        } else {
             RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Lidar configuration invalid: no body_name");
             lidar_publisher_->unlock();
             return;
        }

        // 2. Compute Ray Vectors in Global Frame
        // We need to replicate the start point for each ray for mj_multiRay
        std::vector<mjtNum> ray_pnts(samples_ * 3);

        for (int i = 0; i < samples_; ++i) {
            // Set start point for this ray
            ray_pnts[3 * i + 0] = pos[0];
            ray_pnts[3 * i + 1] = pos[1];
            ray_pnts[3 * i + 2] = pos[2];

            // Rotate to global frame: vec = mat * local_vec
            mju_mulMatVec(ray_vecs_.data() + 3 * i, mat, local_ray_vecs_.data() + 3 * i, 3, 3);
        }

        // 3. Ray Cast
        mj_multiRay(model_, data_, ray_pnts.data(), ray_vecs_.data(), NULL, 1, body_exclude,
                    ray_geomids_.data(), ray_dists_.data(), samples_, sensor_.range_max);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 20, 
            "Lidar raycast done. Origin: [%.2f, %.2f, %.2f], Body Exclude: %d", pos[0], pos[1], pos[2], body_exclude);

        // 4. Publish PointCloud2
        auto& msg = lidar_publisher_->msg_;
        msg.header.stamp = node_->now();
        msg.header.frame_id = sensor_.frame_id;
        msg.height = 1;
        msg.width = samples_;
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(msg.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        int valid_points = 0;
        for (int i = 0; i < samples_; ++i, ++iter_x, ++iter_y, ++iter_z) {
            double dist = ray_dists_[i];

            if (dist == -1.0 || dist < sensor_.range_min || dist > sensor_.range_max) {
                *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
            } else {
                valid_points++;
                *iter_x = static_cast<float>(dist * local_ray_vecs_[3 * i + 0]);
                *iter_y = static_cast<float>(dist * local_ray_vecs_[3 * i + 1]);
                *iter_z = 0.0f;
            }
        }

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
            "Publishing Lidar Cloud. Valid points: %d/%d", valid_points, samples_);

        lidar_publisher_->unlockAndPublish();
    } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Lidar update: Could not acquire lock on publisher.");
    }
}

} // namespace mujoco_ros2_sensors
