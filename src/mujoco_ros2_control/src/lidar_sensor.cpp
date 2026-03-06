#include "mujoco_ros2_sensors/lidar_sensor.hpp"
namespace mujoco_ros2_sensors {

    LidarSensor::LidarSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                           const LidarSensorStruct &sensor, std::atomic<bool>* stop, double frequency) {
        this->nh_ = node;
        this->mujoco_data_ = data;
        this->sensor_ = sensor;

        this->publisher_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("~/lidar", rclcpp::SystemDefaultsQoS());
        this->lidar_publisher_ = std::make_unique<LidarPublisher>(publisher_);
        lidar_publisher_->lock();
        lidar_publisher_->msg_.header.frame_id = sensor_.frame_id;
        lidar_publisher_->msg_.angle_min = sensor_.angle_min;
        lidar_publisher_->msg_.angle_max = sensor_.angle_max;
        lidar_publisher_->msg_.range_min = sensor_.range_min;
        lidar_publisher_->msg_.range_max = sensor_.range_max;

        // Calculate the increment between beams
        size_t num_beams = sensor_.range_sensor_adrs.size();
        if (num_beams > 1) {
            lidar_publisher_->msg_.angle_increment = (sensor_.angle_max - sensor_.angle_min) / (num_beams - 1);
        }
        
        // Pre-allocate the ranges vector to match the number of sensors
        lidar_publisher_->msg_.ranges.resize(num_beams);

        lidar_publisher_->unlock();


        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0 / frequency),
                std::bind(&LidarSensor::update, this));
    }

    void LidarSensor::update() {
        if (lidar_publisher_->trylock()) {
            lidar_publisher_->msg_.header.stamp = nh_->now();

            for (size_t i = 0; i < sensor_.range_sensor_adrs.size(); ++i) {
                int adr = sensor_.range_sensor_adrs[i];
                
                // Get data from MuJoCo's global sensordata array
                double distance = mujoco_data_->sensordata[adr];
                
                // Assign to the message
                lidar_publisher_->msg_.ranges[i] = static_cast<float>(distance);
            }

            lidar_publisher_->unlockAndPublish();
        }

    }
}