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

        // Find the reference body (lidar_link) to transform points into its frame
        int ref_body_id = mj_name2id(model, mjOBJ_BODY, "lidar_link");
        if (ref_body_id == -1) {
            RCLCPP_ERROR(nh_->get_logger(), "Body 'lidar_link' not found in MuJoCo model. Lidar points may be wrong.");
            ref_body_id = 0; // Fallback to world
        }

        // Get reference frame global pose
        const double* ref_pos = data->xpos + 3 * ref_body_id;
        const double* ref_mat = data->xmat + 9 * ref_body_id;

        for (int sensor_id : sensor_.sensor_ids) {
            int site_id = model->sensor_objid[sensor_id];
            
            // Get site global pose
            const double* site_pos = data->site_xpos + 3 * site_id;
            const double* site_mat = data->site_xmat + 9 * site_id;

            // Calculate relative position: P_local = R_ref^T * (P_global - P_ref)
            double rel_pos[3];
            double diff[3] = {site_pos[0] - ref_pos[0], site_pos[1] - ref_pos[1], site_pos[2] - ref_pos[2]};
            mju_mulMatTVec(rel_pos, ref_mat, diff, 3, 3);

            // Calculate relative direction (Z-axis of site): D_local = R_ref^T * (R_site * Z_unit)
            double site_z[3] = {site_mat[2], site_mat[5], site_mat[8]}; // 3rd column of rotation matrix is Z axis
            double rel_dir[3];
            mju_mulMatTVec(rel_dir, ref_mat, site_z, 3, 3);

            beam_origins_.insert(beam_origins_.end(), {rel_pos[0], rel_pos[1], rel_pos[2]});
            beam_directions_.insert(beam_directions_.end(), {rel_dir[0], rel_dir[1], rel_dir[2]});
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