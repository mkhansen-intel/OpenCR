/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/open_manipulator_libs/Linear_Kinematics.h"

using namespace ROBOTIS_MANIPULATOR;
using namespace LINEAR_KINEMATICS;

void Linear::updatePassiveJointValue(Manipulator *manipulator){}

Eigen::MatrixXd Linear::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = ZERO_VECTOR;

  Eigen::Vector3d position_changed = ZERO_VECTOR;
  Eigen::Vector3d orientation_changed = ZERO_VECTOR;
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  int8_t index = 0;
  Name my_name = manipulator->getIteratorBegin()->first;

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getJointAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationToWorld(parent_name) * manipulator->getJointAxis(my_name);
    }

    position_changed = RM_MATH::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionToWorld(tool_name) - manipulator->getComponentPositionToWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void Linear::forward(Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Linear::forward(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Eigen::Vector3d parent_position_to_world, my_position_to_world;
  Eigen::Matrix3d parent_orientation_to_world, my_orientation_to_world;

  if (parent_name == manipulator->getWorldName())
  {
    parent_position_to_world = manipulator->getWorldPosition();
    parent_orientation_to_world = manipulator->getWorldOrientation();
  }
  else
  {
    parent_position_to_world = manipulator->getComponentPositionToWorld(parent_name);
    parent_orientation_to_world = manipulator->getComponentOrientationToWorld(parent_name);
  }

  my_position_to_world = parent_orientation_to_world * manipulator->getComponentRelativePositionToParent(my_name) + parent_position_to_world;
  my_orientation_to_world = parent_orientation_to_world * RM_MATH::rodriguesRotationMatrix(manipulator->getJointAxis(my_name), manipulator->getJointValue(my_name));

  manipulator->setComponentPositionToWorld(my_name, my_position_to_world);
  manipulator->setComponentOrientationToWorld(my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forward(manipulator, child_name);
  }

  // Or 
  // double radius = 0.02818;
  // double motor_angle[3];
  // Pose result_pose;
  
  // result_pose.position(0) = radius * motor_angle[0];
  // result_pose.position(1) = radius * motor_angle[2];  
}

std::vector<double> Linear::inverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  geometricInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<double> Linear::geometricInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<double> target_angle_vector;
  double target_angle[3];       

  double radius = 0.02818;
  double x_limit = 0.300;
  double y_limit = 0.300;

  if (target_pose.position(0) >  x_limit) target_pose.position(0) = x_limit;
  if (target_pose.position(0) < -x_limit) target_pose.position(0) = -x_limit;
  if (target_pose.position(1) >  y_limit) target_pose.position(1) = y_limit;
  if (target_pose.position(1) < -y_limit) target_pose.position(1) = -y_limit;

  target_angle[0] = target_pose.position(0) / radius;
  target_angle[1] = -target_angle[0];
  target_angle[2] = target_pose.position(1) / radius;

  // Set Joint Angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  return target_angle_vector;
}


void Linear::setOption(const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;
  STRING inverse_solver_option = get_arg_[0];
  inverse_solver_option_ = inverse_solver_option;
}


