#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

//bool add(beginner_tutorials::AddTwoInts::Request  &req,
//         beginner_tutorials::AddTwoInts::Response &res)
class ServerNode : public rclcpp::Node
{
public:
    explicit ServerNode(const std::string & service_name)
    : Node("add_two_ints_server")
    {
    // 收到服务请求之后的回调函数
    auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
        {
            (void)request_header;
            RCLCPP_INFO(this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
              request->a, request->b);
            response->sum = request->a + request->b;
        };

    // 创建服务，通过回调函数处理服务请求
    srv_ = create_service<example_interfaces::srv::AddTwoInts>(service_name, handle_add_two_ints);
}

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char * argv[])
{
    //ros::init(argc, argv, "add_two_ints_server");
    rclcpp::init(argc, argv);

    //ros::NodeHandle n;
    auto service_name = std::string("add_two_ints");

    //ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    auto node = std::make_shared<ServerNode>(service_name);

    //ros::spin();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
