#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_current.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include <cstdlib>
#include <dynamixel_sdk_custom_interfaces/msg/detail/set_current__struct.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/detail/get_position__struct.hpp>
#include <memory>
#include <cmath>
#include <rclcpp/publisher.hpp>
#include <chrono>

int getPos(
        const std::shared_ptr<rclcpp::Node> node,
        const rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_get_position
        )
{
    const int jointId = 14;
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
    double msgPos;
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), posFuture) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        msgPos = posFuture.get()->position;
        return msgPos;
    }
    throw std::invalid_argument( "Did not receive position");
}

void setCurrent(
        rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>::SharedPtr publisher_current,
        int current)
{
    dynamixel_sdk_custom_interfaces::msg::SetCurrent msgCurrent;
    msgCurrent.id = 14;
    msgCurrent.current = current;
    publisher_current->publish(msgCurrent);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);


    const std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("position_control");

    const rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>::SharedPtr publisher_current = 
        node->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>("set_current", 10);
    
    const rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_get_position =
        node->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");

    const int Reference = 1500;
    const int Kp = 50;
    const int Kd = 15;
    int previous_error = 0;
    std::chrono::high_resolution_clock::time_point previous_time;
    // Control loop
    while(true)
    {
        const int Pos = getPos(node, client_get_position);
        std::cout << "Position: " << Pos << std::endl;
        int error = Reference - Pos;
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = current_time - previous_time;
        double delta_time = elapsed_time.count();

        double derivative = 0.0;
        if (delta_time > 0.0)
        {
            derivative = (error - previous_error) / delta_time;
        }

        int current = Kp * error + Kd * derivative;
        // Do not allow the current to go out of range
        current = current > 1193 ? 1193 : current;
        std::cout << "Current: " << current << std::endl;

        setCurrent(publisher_current, current);
        previous_error = error;
        previous_time = current_time;
    }

    rclcpp::shutdown();
    return 0;
}

