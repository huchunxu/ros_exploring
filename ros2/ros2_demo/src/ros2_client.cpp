#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

example_interfaces::srv::AddTwoInts_Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client,
  example_interfaces::srv::AddTwoInts_Request::SharedPtr request)
{
    auto result = client->async_send_request(request);
    // 等待服务处理结果
    if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get();
    } else {
        return NULL;
    }
}

int main(int argc, char ** argv)
{
    //ros::init(argc, argv, "add_two_ints_client");
    rclcpp::init(argc, argv);

    //ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("add_two_ints_client");

    //ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    auto topic = std::string("add_two_ints");
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>(topic);

    //beginner_tutorials::AddTwoInts srv;
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 2;
    request->b = 3;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
    }

    //client.call(srv)
    auto result = send_request(node, client, request);
    if (result) {
        RCLCPP_INFO(node->get_logger(), "Result of add_two_ints: %zd", result->sum)
    } else {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.")
    }

    rclcpp::shutdown();
    return 0;
}
