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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "open_manipulator_x.cpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <iostream>
#include <fstream>

using std::placeholders::_1;

class SubscriberForwardKinematics : public rclcpp::Node
{
    public:
        SubscriberForwardKinematics()
            : Node("subscriber_forward_kinematics")
        {
            subscription_joints = this->create_subscription<sensor_msgs::msg::JointState>(
                    "joint_states", 10, std::bind(&SubscriberForwardKinematics::joints_callback, this, _1));

            publisher_pose = this->create_publisher<geometry_msgs::msg::Pose>("pose", 10);
        }

    private:
        Robot openManXRobot;  

        void joints_callback(const sensor_msgs::msg::JointState & msg)
        {
            const std::vector<float> jointValues = { (float)msg.position[0],
                (float)msg.position[1], (float)msg.position[2],
                (float)msg.position[3] };

            Eigen::Matrix<float, 4, 4> endEffectorTransform = openManX::getEndEffectorPose(openManXRobot, jointValues);
            std::cout << endEffectorTransform << std::endl;

            geometry_msgs::msg::Point endEffectorPoint;
            endEffectorPoint.x = endEffectorTransform(0, 3);
            endEffectorPoint.y = endEffectorTransform(1, 3);
            endEffectorPoint.z = endEffectorTransform(2, 3);

            // Append to file
            const char* filename = "data.txt";
            std::ofstream appendFile(filename, std::ios::app);
            if (appendFile.is_open()) {
                appendFile << endEffectorPoint.x << "\t" << endEffectorPoint.y << "\t" << endEffectorPoint.z << std::endl;
                appendFile.close();
                std::cout << "Content appended to the file successfully." << std::endl;
            } else {
                std::cerr << "Error appending to the file!" << std::endl;
            }

            geometry_msgs::msg::Quaternion endEffectorOrientation = 
                rotation_matrix_to_quaternion(endEffectorTransform.block<3, 3>(0, 0));

            geometry_msgs::msg::Pose endEffectorPose;
            endEffectorPose.position = endEffectorPoint;
            endEffectorPose.orientation = endEffectorOrientation;

            publisher_pose->publish(endEffectorPose);
        }  

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joints;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_pose;

        geometry_msgs::msg::Quaternion rotation_matrix_to_quaternion(const Eigen::Matrix<float, 3, 3>& R)
        {
            geometry_msgs::msg::Quaternion quat;

            // Compute the trace of the matrix
            float trace = R(0, 0) + R(1, 1) + R(2, 2);

            // If the trace is positive, calculate the quaternion using the trace-based formula
            if (trace > 0) {
                float s = sqrt(trace + 1.0f) * 2.0f;
                quat.w = 0.25f * s;
                quat.x = (R(2, 1) - R(1, 2)) / s;
                quat.y = (R(0, 2) - R(2, 0)) / s;
                quat.z = (R(1, 0) - R(0, 1)) / s;
            } else {
                // Find the largest diagonal element
                int i = 0;
                if (R(1, 1) > R(0, 0)) i = 1;
                if (R(2, 2) > R(i, i)) i = 2;

                int j = (i + 1) % 3;
                int k = (i + 2) % 3;

                float s = sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0f) * 2.0f;

                quat.w = (R(k, j) - R(j, k)) / s;
                quat.x = 0.25f * s;
                quat.y = (R(j, i) + R(i, j)) / s;
                quat.z = (R(k, i) + R(i, k)) / s;
            }
            return quat;
        }

};

int main(int argc, char * argv[])
{
    const char* filename = "data.txt";

    // Create a new file (this will overwrite the file if it exists)
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << "x\ty\tz" << std::endl;
        outFile.close();
        std::cout << "New file created successfully." << std::endl;
    } else {
        std::cerr << "Error creating file!" << std::endl;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}
