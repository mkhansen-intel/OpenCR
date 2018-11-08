/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Hye-Jong KIM, Darby Lim */

#include "../../include/SCARA/om_scara_kinematics.h"

using namespace ROBOTIS_MANIPULATOR;
using namespace OM_KINEMATICS;

void SCARA::updatePassiveJointValue(Manipulator *manipulator){} // No passive joints for SCARA

Eigen::MatrixXd SCARA::jacobian(Manipulator *manipulator, Name tool_name)
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

void SCARA::forward(Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void SCARA::forward(Manipulator *manipulator, Name component_name)
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
}

std::vector<double> SCARA::inverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

std::vector<double> SCARA::geometricInverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<double> target_angle_vector;

  double target_angle[3];
  double link[3];
  double temp_x[1];
  double temp_y[1];
  double temp_z[1];
  double target_pose_length=0;
  double alpha=0;
  double beta=0;

  // Link Lengths
  link[0] = 0.067f;
  link[1] = 0.067f;
  link[2] = 0.107f;

  // Pose for each set of two joints
  temp_x[0] = target_pose.position(0) + 0.241f;
  temp_y[0] = target_pose.position(1);

  target_pose_length = sqrt(temp_x[0]*temp_x[0] + temp_y[0]*temp_y[0]);
  double theta1=0;
  double theta2=0;
  double error=0;
  double temp_error=1000;

  double result=0;

  // Length of Position Difference and Target Angle
  for (int i=0; i<=900; i++){  
    double theta=(double)i/10;
    alpha = acos((link[1]*link[1]+link[2]*link[2]-link[0]*link[0]  // theta2 = theta3
                  -target_pose_length*target_pose_length
                  +2*link[1]*link[2]*cos(theta*PI/180)) 
                  / (-2*target_pose_length*link[0]));
    beta = acos((link[0]*link[0]+link[1]*link[1]-link[2]*link[2]
                  -target_pose_length*target_pose_length
                  +2*link[0]*link[1]*cos(theta*PI/180)) 
                  / (-2*target_pose_length*link[2]));
    error = abs(alpha + beta - 2*theta*PI/180); 

    // Serial.println("theta");
    // Serial.println(theta * PI/180,5);

    if (error < temp_error){  
      // Serial.println("error");
      // Serial.println(error);
      // result = theta*PI/180;
      result = theta;
      theta1 = acos(-temp_y[0]/target_pose_length) - alpha - PI/2;
      theta2 = theta*PI/180;
      temp_error = error;
    }
  }

  // Serial.println("theta1 and theta2");  
  // Serial.println(theta1); 
  // Serial.println(theta1); 
  // Serial.println(theta2); 
  // Serial.println(theta2);  

  target_angle[0] = theta1;
  target_angle[1] = theta2;
  target_angle[2] = theta2;

  // Serial.println(target_angle[0],5);
  // Serial.println(target_angle[1],5);
  // Serial.println(target_angle[2],5);
  // Serial.flush();

  // Set Joint Angle 
  // target_angle_vector.push_back(-1.2);
  // target_angle_vector.push_back(0.5);
  // target_angle_vector.push_back(1.3);
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  return target_angle_vector;
} 















