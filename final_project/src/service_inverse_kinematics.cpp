
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
#include "geometry_msgs/msg/pose.hpp"
#include "open_manipulator_x.cpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <final_project_msgs/srv/detail/get_joint_values__struct.hpp>
#include "final_project_msgs/srv/get_joint_values.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class ServiceInverseKinematics : public rclcpp::Node
{
public:
  ServiceInverseKinematics()
  : Node("service_inverse_kinematics")
  {
    service_pose = this->create_service<final_project_msgs::srv::GetJointValues>(
            "getJointPositions", 
            std::bind(&ServiceInverseKinematics::pose_callback, this, _1, _2));
  }

private:
  Robot openManXRobot;

  void pose_callback(
        const std::shared_ptr<final_project_msgs::srv::GetJointValues::Request> request,
        const std::shared_ptr<final_project_msgs::srv::GetJointValues::Response> response
    )
  {
      const float endEffectorX = request->pose.position.x;
      const float endEffectorY = request->pose.position.y;
      const float endEffectorZ = request->pose.position.z;

      // Convert quaternion to Rotation matrix
      Eigen::Matrix<float, 3, 3> rotEndEffector = quaternion_to_rotation_matrix(request->pose.orientation);

      const std::vector<float> jointValues = openManX::getJointValues(
              openManXRobot, endEffectorX, endEffectorY, endEffectorZ, 
              rotEndEffector);

      RCLCPP_INFO(this->get_logger(), "Joints: \n'q1: %f\nq2: %f\nq3: %f\nq4: %f'", 
              jointValues[0]*(180/M_PI), jointValues[1]*(180/M_PI), 
              jointValues[2]*(180/M_PI), jointValues[3]*(180/M_PI));

      response->joint_position.joint_name = { "joint 1", "joint 2", "joint 3",
          "joint 4" };
      response->joint_position.position = { jointValues[0], jointValues[1],
          jointValues[2], jointValues[3] };
      response->joint_position.max_velocity_scaling_factor = 0.5;
      response->joint_position.max_accelerations_scaling_factor = 0.5;
  }

  Eigen::Matrix<float, 3, 3> quaternion_to_rotation_matrix(const geometry_msgs::msg::Quaternion& quat)
  {
      // Convert the geometry_msgs::msg::Quaternion to an Eigen::Quaternionf
      Eigen::Quaternionf eigen_quat(quat.w, quat.x, quat.y, quat.z);

      // Convert the quaternion to a 3x3 rotation matrix
      Eigen::Matrix<float, 3, 3> rotation_matrix = eigen_quat.toRotationMatrix();

      return rotation_matrix;
  }

  rclcpp::Service<final_project_msgs::srv::GetJointValues>::SharedPtr service_pose;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceInverseKinematics>());
  rclcpp::shutdown();
  return 0;
}
