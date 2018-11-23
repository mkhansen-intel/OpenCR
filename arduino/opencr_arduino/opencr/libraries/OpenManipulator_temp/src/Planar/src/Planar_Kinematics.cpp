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

#include "../include/open_manipulator_libs/Planar_Kinematics.h"

using namespace ROBOTIS_MANIPULATOR;
using namespace PLANAR_KINEMATICS;

void Planar::updatePassiveJointValue(Manipulator *manipulator){}

Eigen::MatrixXd Planar::jacobian(Manipulator *manipulator, Name tool_name)
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

void Planar::forward(Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Planar::forward(Manipulator *manipulator, Name component_name)
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

  // --------------- OR --------------//
  // double theta2[3] = {45,45,45}; 
  // double temp_x; 
  // double temp_y; 
  // double start_x[3]; 
  // double start_y[3]; 
  // double centre_x, centre_y;
  // double link[3];


  // double error= 10000;
  // double temp_error;
  // double result;
  // // Length of Position Difference and Target Angle
  // for (int i=-300; i<=-300; i++){  
  //   double theta=(double)i/10;

  //   // Link Lengths
  //   link[0] = 0.120f;
  //   link[1] = 0.098f;
  //   link[2] = 0.0366f;

  //   // Start Pose for each set of two joints
  //   for (int i=0; i<3; i++){
  //     start_x[i] = cos(PI*2.0f/3.0f*i) * (-0.1705f + link[0]*sin(theta2[i]*PI/180));
  //     start_y[i] = sin(PI*2.0f/3.0f*i) * (-0.1705f - link[0]*cos(theta2[i]*PI/180));
  //   }
    
  //   Matrix2f alpha = Matrix2f::Zero();
  //   Vector2f beta = Vector2f::Zero();
  //   alpha << start_y[2]-start_y[1], start_y[1]-start_y[2],
  //            start_x[2]-start_x[3], start_x[2]-start_x[1];
  //   beta  << start_x[2]*start_x[2]-start_x[1]*start_x[1]+start_y[2]*start_y[2]-start_y[1]*start_y[1],
  //            start_x[3]*start_x[3]-start_x[2]*start_x[2]+start_y[3]*start_y[3]-start_y[2]*start_y[2];
             
  //   Matrix2f centre = Matrix2f::Zero();

  //   // centre = alpha * beta / 
  //           //  (2*((start_x[2]-start_x[1])*(start_y[3]*start_y[2])
  //               // -(start_y[2]-start_y[1])*(start_x[3]*start_x[2])));


  //   // temp_error = abs(sqrt((centre(0)-start_x[0])^(centre(0)-start_x[0])
  //   //                +(centre(1)-start_y[0])^(centre(1)-start_y[0])) - link[1]);

  //   if (error < temp_error){  
  //     result = theta;
  //     centre_x = centre(0);
  //     centre_y = centre(1);
  //     error = temp_error;
  //   }
  // }  
}

std::vector<double> Planar::inverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  geometricInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<double> Planar::geometricInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<double> target_angle_vector;

  double link[3];
  double start_x[3];    
  double start_y[3];
  double temp_x[3];       
  double temp_y[3];
  double goal_x[3];     
  double goal_y[3];
  Matrix3d goal_orientation;
  double diff_x[3];       
  double diff_y[3];
  double temp_target_angle[3];
  double temp_diff[3];      
  double target_pose_length[3];
  double target_angle[7];

  // Link Lengths
  link[0] = 0.120f;
  link[1] = 0.098f;
  link[2] = 0.0366f;

  // Start Pose for each set of two joints
  for (int i=0; i<3; i++){
    start_x[i] = cos(PI*2.0f/3.0f*i)*(-0.1705f);
    start_y[i] = sin(PI*2.0f/3.0f*i)*(-0.1705f);
  }

  // Goal Pose without tool rotation for each set of two joints
  for (int i=0; i<3; i++){
    temp_x[i] = target_pose.position(0) + cos(PI*2.0f/3.0f*i)*(-link[2]);
    temp_y[i] = target_pose.position(1) + sin(PI*2.0f/3.0f*i)*(-link[2]);
  }

  // Goal Pose for each set of two joints after tool rotation
  goal_orientation = target_pose.orientation;
  if (goal_orientation(0,0) || goal_orientation(0,1) || goal_orientation(0,2)
   || goal_orientation(1,1) || goal_orientation(1,1) || goal_orientation(1,1)
   || goal_orientation(2,1) || goal_orientation(2,1) || goal_orientation(2,1))
  {
    goal_orientation(0,0) = 1;
    goal_orientation(1,1) = 1;
    goal_orientation(2,2) = 1;
  }
  for (int i=0; i<3; i++){
    goal_x[i] = goal_orientation(0,0)*temp_x[i] + goal_orientation(0,1)*temp_y[i];
    goal_y[i] = goal_orientation(1,0)*temp_x[i] + goal_orientation(1,1)*temp_y[i];
    diff_x[i] = goal_x[i] - start_x[i];
    diff_y[i] = goal_y[i] - start_y[i];
    target_pose_length[i] = sqrt(diff_x[i]*diff_x[i] + diff_y[i]*diff_y[i]);
  }

  // Length of Position Difference and Target Angle
  for (int i=0; i<3; i++){
    temp_target_angle[i] = acos((target_pose_length[i]*target_pose_length[i] + link[0]*link[0] - link[1]*link[1]) 
                            / (2*target_pose_length[i]*link[0]));
    temp_diff[i] = sin(-PI*2.0f/3.0f*i)*diff_x[i] + cos(-PI*2.0f/3.0f*i)*diff_y[i];
    target_angle[i] = acos(-temp_diff[i] / target_pose_length[i]) - temp_target_angle[i] - PI/4.0f;
  }

  // Set Joint Angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  for (int i=0; i<3; i++){
    target_angle[i] += PI/4.0f; 
    target_angle[i+3] = acos((-sin(PI*2.0f/3.0f*i)*diff_x[i] + cos(PI*2.0f/3.0f*i)*diff_y[i] 
                              + link[0]*cos(target_angle[i])) / -link[1]) - target_angle[i] - PI*7.0f/12.0f;
  }

  target_angle_vector.push_back(target_angle[3]);
  target_angle_vector.push_back(target_angle[4]);
  target_angle_vector.push_back(target_angle[5]);

  target_angle[3] += PI*7.0f/12.0f;
  // target_angle[6] = acos((target_pose.position(1)-start_y[0]
  //                         -link[0]*sin(target_angle[0])-link[1]*sin(target_angle[0]+target_angle[3]))/link[2])
  //                         -target_angle[0] - target_angle[3];

  if ((target_pose.position(0)-goal_x[0])/link[2] > 1)
    target_pose.position(0) = goal_x[0] + link[2];

  target_angle[6] = acos((target_pose.position(0)-goal_x[0])/link[2]) +PI/2- target_angle[0] - target_angle[3]+PI/3.0f;

  target_angle_vector.push_back(target_angle[6]);

  return target_angle_vector;
}


void Planar::setOption(const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;
  STRING inverse_solver_option = get_arg_[0];
  inverse_solver_option_ = inverse_solver_option;
}


