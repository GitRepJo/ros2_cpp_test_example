// Copyright (c) 2022 Jonas Mahler

#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SubscriberPublisher : public rclcpp::Node
{
public:
  SubscriberPublisher()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "input_topic", 10, std::bind(&SubscriberPublisher::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>(
    "output_topic", 10);
  }
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    if (msg->data == "Hello World"){
      auto message_out = std_msgs::msg::String();
      message_out.data = "Greetings";
      publisher_->publish(message_out);
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberPublisher>());
  rclcpp::shutdown();
  return 0;
}
