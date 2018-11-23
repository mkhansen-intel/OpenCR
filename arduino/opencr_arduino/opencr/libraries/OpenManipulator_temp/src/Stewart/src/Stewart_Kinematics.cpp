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

#include "../include/open_manipulator_libs/Stewart_Kinematics.h"

using namespace ROBOTIS_MANIPULATOR;
using namespace STEWART_KINEMATICS;

void Stewart::updatePassiveJointValue(Manipulator *manipulator){}

Eigen::MatrixXd Stewart::jacobian(Manipulator *manipulator, Name tool_name)
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

void Stewart::forward(Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Stewart::forward(Manipulator *manipulator, Name component_name)
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

std::vector<double> Stewart::inverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  geometricInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<double> Stewart::geometricInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<double> target_angle_vector;

  double temp_angle[6];
  double temp_angle2[6];
  double target_angle[6];
  double link[2];
  double start_x[6];       
  double start_y[6];
  double start_z[6];
  double temp_x[6];       
  double temp_y[6];
  double temp_z[6];
  double target_x[6];       
  double target_y[6];
  double target_z[6];
  double diff_x[6];       
  double diff_y[6];
  double diff_z[6];
  double temp[6];
  double temp2[6];
  double target_pose_length[6];
  Matrix3d goal_orientation;

  // Link Lengths
  link[0] = 0.026f;    // modified the values
  link[1] = 0.1227f;   // modified the values

  // Start pose for each set of two joints
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      start_x[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*(-0.0835f);  // angle :25degrees
      start_y[i] = sin(PI*2.0/3.0*(i/2) - 0.436)*(-0.0835f);  // modified the values
    } 
    else {
      start_x[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*(-0.0835f);  // modified the values
      start_y[i] = sin(PI*2.0/3.0*(i/2) + 0.436)*(-0.0835f);  // modified the values
    }
    start_z[i] = -0.1051; // modified the values
  }  

  // Goal pose for each set of two joints
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      temp_x[i] = target_pose.position(0) + cos(PI*2.0/3.0*(i/2) - 0.911)*(-0.07825f);
      temp_y[i] = target_pose.position(1) + sin(PI*2.0/3.0*(i/2) - 0.911)*(-0.07825f);
    } 
    else {
      temp_x[i] = target_pose.position(0) + cos(PI*2.0/3.0*(i/2) + 0.911)*(-0.07825f);
      temp_y[i] = target_pose.position(1) + sin(PI*2.0/3.0*(i/2) + 0.911)*(-0.07825f);
    }
    temp_z[i] = target_pose.position(2);
  }

  // Goal Pose for each set of two joints after tool rotation
  goal_orientation = target_pose.orientation;
  if (goal_orientation(0,0) && goal_orientation(0,1) && goal_orientation(0,2)
   && goal_orientation(1,0) && goal_orientation(1,1) && goal_orientation(1,2)
   && goal_orientation(2,0) && goal_orientation(2,1) && goal_orientation(2,2))
  {
    goal_orientation(0,0) = 1;
    goal_orientation(1,1) = 1;
    goal_orientation(2,2) = 1;
  }
  for (int i=0; i<6; i++){
    target_x[i] = goal_orientation(0,0)*temp_x[i] + goal_orientation(0,1)*temp_y[i] + goal_orientation(0,2)*temp_z[i];
    target_y[i] = goal_orientation(1,0)*temp_x[i] + goal_orientation(1,1)*temp_y[i] + goal_orientation(1,2)*temp_z[i];
    target_z[i] = goal_orientation(2,0)*temp_x[i] + goal_orientation(2,1)*temp_y[i] + goal_orientation(2,2)*temp_z[i];

  }

  // Pose difference for each set of two joints
  for (int i=0; i<6; i++){
    diff_x[i] = target_x[i] - start_x[i];
    diff_y[i] = target_y[i] - start_y[i];
    diff_z[i] = target_z[i] - start_z[i];
  }

  // for (int i=0; i<3; i++){
  //   temp[i] = diff_x[i]*cos(PI*2.0/3.0*i)+diff_y[i]*sin(PI*2.0/3.0*i);
  //   temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);
  //   temp_angle[i] = acos((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
  //                   / (2.0*link[0]*temp2[i]));
  //   temp_angle2[i] = acos(temp[i] / temp2[i]);
  //   target_angle[i] = +temp_angle[i] - temp_angle2[i];  
  // }

  // for (int i=0; i<6; i++){
  // Serial.println(diff_x[i]*diff_x[i] + diff_y[i]*diff_y[i] + diff_z[i]*diff_z[i],4);
  // }
  for (int i=0; i<6; i++){
    // if (i%2 == 0) {
    //   temp[i] = diff_x[i]*cos(PI*2.0/3.0*(i/2) - 0.436)+diff_y[i]*sin(PI*2.0/3.0*(i/2) - 0.436);  
    // } 
    // else {
    //   temp[i] = diff_x[i]*cos(PI*2.0/3.0*(i/2) + 0.436)+diff_y[i]*sin(PI*2.0/3.0*(i/2) + 0.436);  
    // }
    if (i%2 == 0) {
      temp[i] = -diff_x[i]*sin(PI*2.0/3.0*(i/2) - 0.436)+diff_y[i]*cos(PI*2.0/3.0*(i/2) - 0.436);  
    } 
    else {
      temp[i] = -diff_x[i]*sin(PI*2.0/3.0*(i/2) + 0.436)+diff_y[i]*cos(PI*2.0/3.0*(i/2) + 0.436);  
    }
    temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);

    temp_angle[i] = asin((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
                    / (2.0*link[0]*temp2[i]));
    if (i%2 == 0) {
      temp_angle2[i] = asin(temp[i] / temp2[i]);
    } 
    else {
      temp_angle2[i] = -asin(temp[i] / temp2[i]);
    }
    Serial.println(temp[i],4);
    Serial.println(temp2[i],4);

    target_angle[i] = -temp_angle[i] + temp_angle2[i];  
  }

  // Set Joint Angle 
  Serial.println(temp_angle[0],4);
  Serial.println(temp_angle2[0],4);
  Serial.println(target_angle[0],4);
  Serial.println(-target_angle[1],4);
  Serial.println(target_angle[2],4);
  Serial.println(-target_angle[3],4);
  Serial.println(target_angle[4],4);
  Serial.println(-target_angle[5],4);

  // start_x[0] = cos(PI*2.0/3.0*(0/2) - 0.436)*(-0.0835f) - sin(PI*2.0/3.0*(0/2) - 0.436)*(-0.026f);
  // start_y[0] = sin(PI*2.0/3.0*(0/2) - 0.436)*(-0.0835f) + cos(PI*2.0/3.0*(0/2) - 0.436)*(-0.026f);
  // diff_x[0] = target_x[0] - start_x[0];
  // diff_y[0] = target_y[0] - start_y[0];

  // double zvalue;
  // zvalue = sqrt(link[1]*link[1] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0]);
  // Serial.println(zvalue,4);
  
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(-target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);
  target_angle_vector.push_back(-target_angle[3]);
  target_angle_vector.push_back(target_angle[4]);
  target_angle_vector.push_back(-target_angle[5]);
  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);

  return target_angle_vector;
}


void Stewart::setOption(const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;
  STRING inverse_solver_option = get_arg_[0];
  inverse_solver_option_ = inverse_solver_option;
}


