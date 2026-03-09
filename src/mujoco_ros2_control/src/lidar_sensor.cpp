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
    min_theta_ = node_->declare_parameter<double>("min_theta", -M_PI);
    max_theta_ = node_->declare_parameter<double>("max_theta", M_PI);
    min_phi_ = node_->declare_parameter<double>("min_phi", -M_PI*4.5/180);
    max_phi_ = node_->declare_parameter<double>("max_phi", M_PI/3);
    
    h_samples_ = node_->declare_parameter<int>("h_samples", 360);
    v_samples_ = node_->declare_parameter<int>("v_samples", 20);

    // Allow overriding range limits from ROS params, otherwise use defaults from struct
    if (node_->has_parameter("range_min")) {
        sensor_.range_min = node_->get_parameter("range_min").as_double();
    }
    if (node_->has_parameter("range_max")) {
        sensor_.range_max = node_->get_parameter("range_max").as_double();
    }

    // Resize buffers for mj_multiRay
    ray_vecs_.resize(h_samples_ * v_samples_ * 3);
    local_ray_vecs_.resize(h_samples_ * v_samples_ * 3);
    ray_dists_.resize(h_samples_ * v_samples_);
    ray_geomids_.resize(h_samples_ * v_samples_);

    // Precompute local vectors
    double theta_step = (h_samples_ > 1) ? (max_theta_ - min_theta_) / (h_samples_ - 1) : 0.0;
    double phi_step = (v_samples_ > 1) ? (max_phi_ - min_phi_) / (v_samples_ - 1) : 0.0;
    for (int i = 0; i < h_samples_; ++i) {
        for (int j = 0; j < v_samples_; ++j) {
            double theta = min_theta_ + i * theta_step;
            double phi = min_phi_ + j * phi_step;
            local_ray_vecs_[3 * (j*h_samples_ + i) + 0] = std::cos(theta)*std::cos(phi);
            local_ray_vecs_[3 * (j*h_samples_ + i) + 1] = std::sin(theta)*std::cos(phi);
            local_ray_vecs_[3 * (j*h_samples_ + i) + 2] = std::sin(phi);

        }
    }

    RCLCPP_INFO(node_->get_logger(), "Lidar initialized on frame '%s' with %d samples. Range: [%.2f, %.2f]",
                sensor_.frame_id.c_str(), h_samples_ * v_samples_, sensor_.range_min, sensor_.range_max);

    timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0 / frequency),
            std::bind(&LidarSensor::update, this));
}

void LidarSensor::update()
{
    if (lidar_publisher_->trylock()) {
        // Get Sensor Pose - in this case, it is the baselink, except, it is offset 0.2 m high
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

        // Compute Ray Vectors in Global Frame
        std::vector<mjtNum> ray_pnts(v_samples_ * h_samples_ * 3);

        for (int i = 0; i < v_samples_ * h_samples_; ++i) {
            // Set start point for this ray
            ray_pnts[3 * i + 0] = pos[0];
            ray_pnts[3 * i + 1] = pos[1];
            ray_pnts[3 * i + 2] = pos[2]+0.2; // This is the 0.2 m offset.

            // Rotate to global frame: vec = mat * local_vec
            mju_mulMatVec(ray_vecs_.data() + 3 * i, mat, local_ray_vecs_.data() + 3 * i, 3, 3);
        }

        // Ray Cast
        mj_multiRay(model_, data_, ray_pnts.data(), ray_vecs_.data(), NULL, 1, body_exclude,
                    ray_geomids_.data(), ray_dists_.data(), v_samples_ * h_samples_, sensor_.range_max);

        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 40, 
        //     "Lidar raycast done. Origin: [%.2f, %.2f, %.2f], Body Exclude: %d", pos[0], pos[1], pos[2], body_exclude);

        // Publish the point cloud
        auto& msg = lidar_publisher_->msg_;
        msg.header.stamp = node_->now();
        msg.header.frame_id = sensor_.frame_id;
        msg.height = 1;
        msg.width = v_samples_ * h_samples_;
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(msg.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        int valid_points = 0;
        for (int i = 0; i < v_samples_ * h_samples_; ++i, ++iter_x, ++iter_y, ++iter_z) {
            double dist = ray_dists_[i];

            if (dist == -1.0 || dist > sensor_.range_max) {
                dist = sensor_.range_max;
            } else if (dist < sensor_.range_min) {
                dist = sensor_.range_min;
            }

            valid_points++; // Used for debugging
            *iter_x = static_cast<float>(dist * local_ray_vecs_[3 * i + 0]);
            *iter_y = static_cast<float>(dist * local_ray_vecs_[3 * i + 1]);
            *iter_z = static_cast<float>(dist * local_ray_vecs_[3 * i + 2]);
        }

        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 40, 
        //     "Publishing Lidar Cloud. Valid points: %d/%d", valid_points, samples_);

        lidar_publisher_->unlockAndPublish();
    } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Lidar update: Could not acquire lock on publisher.");
    }
}

} // namespace mujoco_ros2_sensors
