#ifndef MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mujoco_ros2_sensors {
    struct LidarSensorStruct {
        std::string frame_id;
        std::vector<int> range_sensor_adrs; // Addresses of each rangefinder in the array
        double angle_min;
        double angle_max;
        double range_min;
        double range_max;

        bool isValid() const {
            return !range_sensor_adrs.empty();
        }
    };
    class LidarSensor {
    public:
        LidarSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                   const LidarSensorStruct &sensor, std::atomic<bool>* stop, double frequency);
    private:
        void update();

        std::atomic<bool>* stop_;

        rclcpp::Node::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the ROS 2 timer object used for scheduling periodic updates.

        // fallback to pose publisher when position or orientation is missing
        using LidarPublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>;
        using LidarPublisherPtr = std::unique_ptr<LidarPublisher>;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        LidarPublisherPtr lidar_publisher_;

        mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.

        LidarSensorStruct sensor_;
    };
}
#endif //MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
