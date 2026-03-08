#include "mujoco_ros2_sensors/lidar_sensor.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
namespace mujoco_ros2_sensors {

    LidarSensor::LidarSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                           const LidarSensorStruct &sensor, std::atomic<bool>* stop, double frequency) {
        this->nh_ = node;
        this->mujoco_data_ = data;
        this->sensor_ = sensor;

        this->publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lidar", rclcpp::SystemDefaultsQoS());
        this->lidar_publisher_ = std::make_unique<LidarPublisher>(publisher_);

        // Pre-calculate beam geometry
        size_t num_beams = sensor_.sensor_ids.size();
        beam_origins_.reserve(3 * num_beams);
        beam_directions_.reserve(3 * num_beams);

        for (int sensor_id : sensor_.sensor_ids) {
            int site_id = model->sensor_objid[sensor_id];
            // site_pos and site_quat are local to the parent body (lidar_head/lidar_link)
            const double* pos = model->site_pos + 3 * site_id;
            const double* quat = model->site_quat + 4 * site_id;

            // MuJoCo rangefinders point along the site's Z-axis.
            // We rotate the vector (0, 0, 1) by the site's quaternion to get the direction.
            double vec[3] = {0, 0, 1};
            double dir[3];
            mju_rotVecQuat(dir, vec, quat);

            beam_origins_.insert(beam_origins_.end(), {pos[0], pos[1], pos[2]});
            beam_directions_.insert(beam_directions_.end(), {dir[0], dir[1], dir[2]});
        }


        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0 / frequency),
                std::bind(&LidarSensor::update, this));
    }

    void LidarSensor::update() {
        if (lidar_publisher_->trylock()) {
            auto& msg = lidar_publisher_->msg_;
            msg.header.stamp = nh_->now();
            msg.header.frame_id = "lidar_link";
            msg.height = 1;
            msg.width = sensor_.range_sensor_adrs.size();
            msg.is_dense = false;
            msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(msg.width);

            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

            for (size_t i = 0; i < sensor_.range_sensor_adrs.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
                int adr = sensor_.range_sensor_adrs[i];
                double distance = mujoco_data_->sensordata[adr];

                if (distance < sensor_.range_min || distance > sensor_.range_max) {
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                } else {
                    *iter_x = static_cast<float>(beam_origins_[3*i] + beam_directions_[3*i] * distance);
                    *iter_y = static_cast<float>(beam_origins_[3*i+1] + beam_directions_[3*i+1] * distance);
                    *iter_z = static_cast<float>(beam_origins_[3*i+2] + beam_directions_[3*i+2] * distance);
                }
            }

            lidar_publisher_->unlockAndPublish();
        }

    }
}