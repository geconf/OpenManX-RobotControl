// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_x.cpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <final_project_msgs/srv/detail/get_end_effector_velocities__struct.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "final_project_msgs/srv/get_joint_velocities.hpp"
#include "final_project_msgs/srv/get_end_effector_velocities.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class ServicesVelocityKinematics : public rclcpp::Node
{
    public:
        ServicesVelocityKinematics()
            : Node("services_velocity_kinematics")
        {
            ServiceGetEndEffectorVelocity = this->
                create_service<final_project_msgs::srv::GetEndEffectorVelocities>(
                        "getEndEffectorVelocity", 
                        std::bind(&ServicesVelocityKinematics::getEndEffectorVelocity, this, _1, _2));

            ServiceGetJointVelocity = this->
                create_service<final_project_msgs::srv::GetJointVelocities>(
                        "getJointVelocity", 
                        std::bind(&ServicesVelocityKinematics::getJointVelocity, this, _1, _2));
                    
            SubscriptionJoints = this->create_subscription<sensor_msgs::msg::JointState>(
                            "joint_states", 10, std::bind(&ServicesVelocityKinematics::getJoints, this, _1));
        }

    private:
        Robot openManXRobot;
        std::vector<float> JointPosition = { 0, 0, 0, 0 };

        void getEndEffectorVelocity(
                const std::shared_ptr<final_project_msgs::srv::GetEndEffectorVelocities::Request> request,
                const std::shared_ptr<final_project_msgs::srv::GetEndEffectorVelocities::Response> response
        )
        {
            const std::vector<float> jointVelocity = request->joint_velocity.data;
            Eigen::Vector<float, 6> twist = openManX::GetTwist(
                  openManXRobot,
                  JointPosition,
                  jointVelocity);

            geometry_msgs::msg::Twist msgTwist;

            msgTwist.linear.x = twist[0];
            msgTwist.linear.y = twist[1];
            msgTwist.linear.z = twist[2];
            msgTwist.angular.x = twist[3];
            msgTwist.angular.y = twist[4];
            msgTwist.angular.z = twist[5];

            response->end_effector_velocity = msgTwist;
            std::cout << "q1: " << JointPosition[0] << std::endl;
            std::cout << "q2: " << JointPosition[1] << std::endl;
            std::cout << "q3: " << JointPosition[2] << std::endl;
            std::cout << "q4: " << JointPosition[3] << std::endl;
        }

        void getJointVelocity(
                const std::shared_ptr<final_project_msgs::srv::GetJointVelocities::Request> request,
                const std::shared_ptr<final_project_msgs::srv::GetJointVelocities::Response> response
                )
        {
            const geometry_msgs::msg::Twist msgTwist = request->end_effector_velocity;
            Eigen::Vector<float, 6> twist;
            twist[0] = msgTwist.linear.x;
            twist[1] = msgTwist.linear.y;
            twist[2] = msgTwist.linear.z;
            twist[3] = msgTwist.angular.x;
            twist[4] = msgTwist.angular.y;
            twist[5] = msgTwist.angular.z;

            Eigen::Vector<float, 4> jointVelocity = openManX::GetJointVelocity(
                  openManXRobot,
                  JointPosition,
                  twist);

            std::vector<float> msgJointVelocity = { 0, 0, 0, 0 };
            for (uint i = 0; i < openManXRobot.dhParams_.getNumberOfJoints(); i++)
            {
                msgJointVelocity[i] = jointVelocity[i];
            }

            response->joint_velocity.data = msgJointVelocity;
            }

        void getJoints(const sensor_msgs::msg::JointState & msg)
        {
            JointPosition = { 
                (float)msg.position[0],
                (float)msg.position[1], 
                (float)msg.position[2],
                (float)msg.position[3] };
        }  

        rclcpp::Service<final_project_msgs::srv::GetEndEffectorVelocities>::SharedPtr ServiceGetEndEffectorVelocity;
        rclcpp::Service<final_project_msgs::srv::GetJointVelocities>::SharedPtr ServiceGetJointVelocity;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr SubscriptionJoints;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServicesVelocityKinematics>());
    rclcpp::shutdown();
    return 0;
}
