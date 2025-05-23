#include "pong.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PongNode>());
    rclcpp::shutdown();
    return 0;
}


PongNode::PongNode()
            : Node("pong_node")
            , waitForPingNode_(true)
{
    ping_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "ping", 10, std::bind(&PongNode::ping_callback, this, _1));
    pong_publisher_ = this->create_publisher<std_msgs::msg::String>("pong", 10);

    timer_ = this->create_wall_timer(3s, std::bind(&PongNode::timer_callback, this));

    auto message = std_msgs::msg::String();
    message.data = "Pong ready";
    pong_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Waiting for ping_node...");
}

void PongNode::PongNode::timer_callback()
{
    if (waitForPingNode_)
    {
        auto message = std_msgs::msg::String();
        message.data = "Pong ready";
        pong_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Waiting for ping_node...");
    }
}

void PongNode::ping_callback(const std_msgs::msg::String & msg)
{
    if (waitForPingNode_)
    {
        RCLCPP_INFO(this->get_logger(), "ping_node detected.");
        waitForPingNode_ = false;
    }

    //RCLCPP_INFO(this->get_logger(), "Received: '%s', sending back...", msg.data.c_str());
    pong_publisher_->publish(msg);
}

