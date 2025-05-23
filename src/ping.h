#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "Measurement.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class PingNode : public rclcpp::Node
{
    public:
        PingNode();
        void printResults();

    private:
        void timer_callback();
        void pong_callback(const std_msgs::msg::String & msg);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pong_subscriber_;
        size_t count_;
        bool active_;
        Measurement measurement_;
};
