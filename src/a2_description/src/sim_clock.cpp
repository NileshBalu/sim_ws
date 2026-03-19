#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

class SimClockNode : public rclcpp::Node {
public:
    SimClockNode() : Node("sim_clock_publisher") {
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        // Use steady_clock to get a reliable elapsed time starting from 0
        start_time_ = std::chrono::steady_clock::now();
        
        // Publish clock at 100 Hz (10 ms)
        clock_timer_ = this->create_wall_timer(
            10ms, std::bind(&SimClockNode::timer_callback, this));
    }

private:
    void timer_callback() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_time_).count();

        auto msg = rosgraph_msgs::msg::Clock();
        msg.clock.sec = static_cast<int32_t>(elapsed / 1000000000LL);
        msg.clock.nanosec = static_cast<uint32_t>(elapsed % 1000000000LL);
        clock_pub_->publish(msg);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::TimerBase::SharedPtr clock_timer_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimClockNode>());
    rclcpp::shutdown();
    return 0;
}