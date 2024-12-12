#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_current.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include <cstdlib>
#include <dynamixel_sdk_custom_interfaces/srv/detail/get_position__struct.hpp>
#include <memory>
#include <cmath>

int getPos(
        const std::shared_ptr<rclcpp::Node> node,
        const rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_get_position
        )
{
    const int jointId = 4;
    const auto getPosition = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
    getPosition->id = jointId;

    auto posFuture = client_get_position->async_send_request(getPosition);

    // Wait for service
    while (!client_get_position->wait_for_service())
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");   
            abort();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // Wait for result
    int msgPos;
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), posFuture) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        msgPos = posFuture.get()->position;
        return msgPos;
    }
    throw std::invalid_argument( "Did not receive position");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);


    const std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("position_control");

    const rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>::SharedPtr publisher_current = 
        node->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>("set_current", 10);
    
    const rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_get_position =
        node->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");

    const double reference = 45;

    int pos = 0;
    while(true)
    {
        pos = getPos(node, client_get_position);
        std::cout << pos << std::endl;
    }
    /*
    uint current = 0;
    const dynamixel_sdk_custom_interfaces::msg::SetCurrent::SharedPtr msgCurrent;
    msgCurrent->id = 4;
    msgCurrent->current = current;
    */

    rclcpp::shutdown();
    return 0;
}

