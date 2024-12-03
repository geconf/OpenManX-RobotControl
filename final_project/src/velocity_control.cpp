#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "final_project_msgs/srv/get_joint_velocities.hpp"

#include <chrono>
#include <cstdlib>
#include <final_project_msgs/srv/detail/get_joint_velocities__struct.hpp>
#include <memory>
#include <eigen3/Eigen/Dense>

using namespace std::chrono_literals;
/*
void callback(const rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future){}
*/

uint setJointPos(
        std::shared_ptr<rclcpp::Node> node,
        rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr clientJointPos,
        Eigen::Vector<double, 5> jointPos,
        double pathTime) 
{
    auto setJointPos = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    setJointPos->planning_group = "";
    setJointPos->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4", "gripper"};
    std::vector<double> msgJointPos(jointPos.data(), jointPos.data() + jointPos.size());
    setJointPos->joint_position.position = msgJointPos;
    setJointPos->path_time = pathTime;

    while (!clientJointPos->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");   
      return 0;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    }
    auto result = clientJointPos->async_send_request(setJointPos);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
    } 
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
    return 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);


    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("velocity_control");

    rclcpp::Client<final_project_msgs::srv::GetJointVelocities>::SharedPtr clientJointVelocity =
    node->create_client<final_project_msgs::srv::GetJointVelocities>("getJointVelocity");

    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr clientJointPos =
    node->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0.1; // Reference in +y direction
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    Eigen::Vector<double, 5> jointPos = { 0, 0, 0, 0, 0 };
    Eigen::Vector<double, 5> jointPosOld = { 0, 0, 0, 0, 0 };

    // Set initial position
    const double initPathTime = 3;
    setJointPos(node, clientJointPos, jointPos, initPathTime);

    while (true)
    {
        std::cout << "hello POST LOOP" << std::endl;
        auto getJointVelocity = std::make_shared<final_project_msgs::srv::GetJointVelocities::Request>();
        getJointVelocity->end_effector_velocity = twist;

        auto jointVelocityFuture = clientJointVelocity->async_send_request(getJointVelocity);

        // Wait for the result
        std::vector<float> msgJointVelocity;
        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), jointVelocityFuture) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            msgJointVelocity = jointVelocityFuture.get()->joint_velocity.data;
        }

        // Convert msg to jointVelocity
        Eigen::Vector<double, 5> jointVelocity;
        for (int i = 0; i < 5; ++i) 
        {
            jointVelocity[i] = static_cast<double>(msgJointVelocity[i]); 
        }
        // Calculate the new jointPos
        jointPos = jointPosOld + jointVelocity*0.1;
        // Move it
        setJointPos(node, clientJointPos, jointPos, 0.1);
        // Update jointPosOld
        jointPosOld = jointPos;
    }

    rclcpp::shutdown();
    return 0;
}

