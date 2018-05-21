#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
    //ros::init(argc, argv, "talker");
    rclcpp::init(argc, argv);

    //ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("talker");

    // 配置质量服务原则，ROS2针对以下几种应用提供了默认的配置：
    // publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    // 配置QoS中历史数据的缓存深度
    custom_qos_profile.depth = 7;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", custom_qos_profile);

    //ros::Rate loop_rate(10);
    rclcpp::WallRate loop_rate(2);

    auto msg = std::make_shared<std_msgs::msg::String>();
    auto i = 1;

    //while (ros::ok())
    while (rclcpp::ok()) 
    {
        msg->data = "Hello World: " + std::to_string(i++);
        std::cout << "Publishing: '" << msg->data << "'" << std::endl;

        //chatter_pub.publish(msg);
        chatter_pub->publish(msg);

        //ros::spinOnce();
        rclcpp::spin_some(node);

        //loop_rate.sleep();
        loop_rate.sleep();
    }

    return 0;
}
