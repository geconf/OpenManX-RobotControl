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
    ServiceGetEndEffectorVelocities = this->
        create_service<final_project_msgs::srv::GetEndEffectorVelocities>(
                "getEndEffectorVelocities", 
                std::bind(&ServicesVelocityKinematics::getEndEffectorVelocities, this, _1, _2));

    ServiceGetJointVelocities = this->
        create_service<final_project_msgs::srv::GetJointVelocities>(
                "getJointVelocities", 
                std::bind(&ServicesVelocityKinematics::getJointVelocities, this, _1, _2));
  }

private:
  Robot openManXRobot;

  void getEndEffectorVelocities(
        const std::shared_ptr<final_project_msgs::srv::GetEndEffectorVelocities::Request> request,
        const std::shared_ptr<final_project_msgs::srv::GetEndEffectorVelocities::Response> response
    )
  {
      const std::vector<float> qDot = request->joint_velocity.data;
      std::vector<float> q = { 9, 9 ,4 ,4 };
      Eigen::Vector<float, 6> twist = openManX::GetTwist(
              openManXRobot,
              q,
              qDot);

      geometry_msgs::msg::Twist msgTwist;

      msgTwist.linear.x = twist[0];
      msgTwist.linear.y = twist[1];
      msgTwist.linear.z = twist[2];
      msgTwist.angular.x = twist[3];
      msgTwist.angular.y = twist[4];
      msgTwist.angular.z = twist[5];

      response->end_effector_velocity = msgTwist;
  }

  void getJointVelocities(
        const std::shared_ptr<final_project_msgs::srv::GetJointVelocities::Request> request,
        const std::shared_ptr<final_project_msgs::srv::GetJointVelocities::Response> response
    )
  {
      const std::vector<float> qDot = request->joint_velocity.data;
      std::vector<float> q = { 9, 9 ,4 ,4 };
      Eigen::Vector<float, 6> twist = openManX::GetTwist(
              openManXRobot,
              q,
              qDot);

      geometry_msgs::msg::Twist msgTwist;

      msgTwist.linear.x = twist[0];
      msgTwist.linear.y = twist[1];
      msgTwist.linear.z = twist[2];
      msgTwist.angular.x = twist[3];
      msgTwist.angular.y = twist[4];
      msgTwist.angular.z = twist[5];

      //response->end_effector_velocity = msgTwist;
  }

  rclcpp::Service<final_project_msgs::srv::GetEndEffectorVelocities>::SharedPtr ServiceGetEndEffectorVelocities;
  rclcpp::Service<final_project_msgs::srv::GetJointVelocities>::SharedPtr ServiceGetJointVelocities;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServicesVelocityKinematics>());
  rclcpp::shutdown();
  return 0;
}
