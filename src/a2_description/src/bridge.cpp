#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/low_state.hpp"


class A2Bridge : public rclcpp::Node {
public:
  A2Bridge() : Node("a2_bridge_node") {
    // Joint state publisher
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // IMU publisher
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    // Lowstate subscriber
    lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, std::bind(&A2Bridge::lowstate_callback, this, std::placeholders::_1));

    // Joint names - from the URDF file
    joint_names_ = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",    
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };
  }

private:
  void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg) {
    auto now = this->get_clock()->now();

    auto joint_states_msg = sensor_msgs::msg::JointState();
    joint_states_msg.header.stamp = now;
    joint_states_msg.name = joint_names_;

    // Map the first 12 motors to the 12 active joints
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_states_msg.position.push_back(msg->motor_state[i].q);
      joint_states_msg.velocity.push_back(msg->motor_state[i].dq);
      joint_states_msg.effort.push_back(msg->motor_state[i].tau_est);
    }
    joint_states_pub_->publish(joint_states_msg);

    
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link"; // Must match URDF link name

    imu_msg.orientation.w = msg->imu_state.quaternion[0];
    imu_msg.orientation.x = msg->imu_state.quaternion[1];
    imu_msg.orientation.y = msg->imu_state.quaternion[2];
    imu_msg.orientation.z = msg->imu_state.quaternion[3];

    imu_msg.angular_velocity.x = msg->imu_state.gyroscope[0];
    imu_msg.angular_velocity.y = msg->imu_state.gyroscope[1];
    imu_msg.angular_velocity.z = msg->imu_state.gyroscope[2];

    imu_msg.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imu_msg.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imu_msg.linear_acceleration.z = msg->imu_state.accelerometer[2];

    imu_pub_->publish(imu_msg);
  }

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2Bridge>());
  rclcpp::shutdown();
  return 0;
}