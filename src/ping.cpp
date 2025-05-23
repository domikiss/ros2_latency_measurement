#include "ping.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pingNode = std::make_shared<PingNode>();
  rclcpp::spin(pingNode);
  pingNode->printResults();
  rclcpp::shutdown();
  return 0;
}

/***********************************************************************************/

PingNode::PingNode()
            : Node("ping_node")
            , count_(0)
            , active_(false)
{
    ping_publisher_ = this->create_publisher<std_msgs::msg::String>("ping", 10);
    pong_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                    "pong", 10, std::bind(&PingNode::pong_callback, this, _1));

    timer_ = this->create_wall_timer(200ms, std::bind(&PingNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for pong_node...");
}


void PingNode::printResults()
{
    std::cout << std::endl;
    std::cout << "Messages sent:     " << measurement_.getDataNum() << std::endl;
    std::cout << "Messages received: " << measurement_.getReceivedNum() << std::endl;
    std::cout << "Message loss rate: " << measurement_.getLostRate()*100 << "%" << std::endl;
    std::cout << "Roundtrip delay:" << std::endl;
    std::cout << "      - average:   " << measurement_.getAvgDelay() << " ms" << std::endl;
    std::cout << "      - std. dev.: " << measurement_.getDelayStdDev() << " ms" << std::endl;
    std::cout << "      - min:       " << measurement_.getMinDelay() << " ms" << std::endl;
    std::cout << "      - max:       " << measurement_.getMaxDelay() << " ms" << std::endl << std::endl;;

}

void PingNode::timer_callback()
{
    if (active_)
    {
        auto message = std_msgs::msg::String();
        message.data = std::to_string(count_++);
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        chrono::high_resolution_clock::time_point sentTime = chrono::high_resolution_clock::now();
        ping_publisher_->publish(message);

        MeasurementData dataPoint(message.data, sentTime);
        measurement_.addData(dataPoint);
    }
}

void PingNode::pong_callback(const std_msgs::msg::String & msg)
{
    chrono::high_resolution_clock::time_point receivedTime = chrono::high_resolution_clock::now();

    if (msg.data == "Pong ready")
    {
        active_ = true;
        RCLCPP_INFO(this->get_logger(), "pong_node detected");
    }
    else
    {
        size_t messageId = std::stol(msg.data);
        measurement_.setReceivedStatus(messageId,receivedTime);
        RCLCPP_INFO(this->get_logger(),
                    "Received back: '%s', delay: %g ms",
                    msg.data.c_str(),
                    measurement_.getData(messageId).getDelayMs());
    }
    
}

