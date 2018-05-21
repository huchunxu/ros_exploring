#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::cout << "I heard: [" << msg->data << "]" << std::endl;
}

int main(int argc, char * argv[])
{
    //ros::init(argc, argv, "listener");
    rclcpp::init(argc, argv);

    //ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("listener");

    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    auto sub = node->create_subscription<std_msgs::msg::String>(
    "chatter", chatterCallback, rmw_qos_profile_default);

    //ros::spin();
    rclcpp::spin(node);

    return 0;
}
