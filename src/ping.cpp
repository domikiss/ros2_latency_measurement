#include "ping.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto pingNode = std::make_shared<PingNode>();
    rclcpp::spin(pingNode);

    pingNode->printResults();

    if (argc > 1)
    {
        std::string logName(argv[1]);
        pingNode->writeDataToFile(logName + ".csv");
        pingNode->writeResultsToFile(logName + ".txt");
    }

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
    measurement_.printStatistics();
    std::cout << std::endl;
}


void PingNode::writeResultsToFile(std::string filename)
{
    measurement_.writeStatisticsToFile(filename);
}


void PingNode::writeDataToFile(std::string filename)
{
    measurement_.writeRawDataToFile(filename);
}


void PingNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    if (!active_)
    {
        message.data = std::to_string(count_);
        ping_publisher_->publish(message);

    }
    else
    {
        message.data = std::to_string(count_++);

        chrono::high_resolution_clock::time_point sentTime = chrono::high_resolution_clock::now();
        ping_publisher_->publish(message);

        MeasurementData dataPoint(message.data, sentTime);
        measurement_.addData(dataPoint);
    }
}


void PingNode::pong_callback(const std_msgs::msg::String & msg)
{
    chrono::high_resolution_clock::time_point receivedTime = chrono::high_resolution_clock::now();

    if (!active_)
    {
        active_ = true;
        RCLCPP_INFO(this->get_logger(), "pong_node detected.");
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

