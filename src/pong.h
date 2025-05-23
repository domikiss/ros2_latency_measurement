#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PongNode : public rclcpp::Node
{
    public:
        PongNode();

    private:
        void timer_callback();
        void ping_callback(const std_msgs::msg::String & msg);

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ping_subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pong_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool waitForPingNode_;

};
