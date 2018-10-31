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

#include "../../include/open_manipulator/OMKinematics.h"

using namespace Eigen;
using namespace OPEN_MANIPULATOR;
using namespace OM_KINEMATICS;

MatrixXf Chain::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(manipulator));

  Vector3f joint_axis = ZERO_VECTOR;

  Vector3f position_changed = ZERO_VECTOR;
  Vector3f orientation_changed = ZERO_VECTOR;
  VectorXf pose_changed = VectorXf::Zero(6);

  int8_t index = 0;
  Name my_name = getIteratorBegin(manipulator)->first;

  for (int8_t size = 0; size < getDOF(manipulator); size++)
  {
    Name parent_name = getComponentParentName(manipulator, my_name);
    if (parent_name == getWorldName(manipulator))
    {
      joint_axis = getWorldOrientation(manipulator) * getComponentJointAxis(manipulator, my_name);
    }
    else
    {
      joint_axis = getComponentOrientationToWorld(manipulator, parent_name) * getComponentJointAxis(manipulator, my_name);
    }

    position_changed = OM_MATH::skewSymmetricMatrix(joint_axis) *
                       (getComponentPositionToWorld(manipulator, tool_name) - getComponentPositionToWorld(manipulator, my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = getComponentChildName(manipulator, my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void Chain::forward(OM_MANAGER::Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Chain::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = getComponentParentName(manipulator, my_name);
  int8_t number_of_child = getComponentChildName(manipulator, my_name).size();

  Vector3f parent_position_to_world, my_position_to_world;
  Matrix3f parent_orientation_to_world, my_orientation_to_world;

  if (parent_name == getWorldName(manipulator))
  {
    parent_position_to_world = getWorldPosition(manipulator);
    parent_orientation_to_world = getWorldOrientation(manipulator);
  }
  else
  {
    parent_position_to_world = getComponentPositionToWorld(manipulator, parent_name);
    parent_orientation_to_world = getComponentOrientationToWorld(manipulator, parent_name);
  }

  my_position_to_world = parent_orientation_to_world * getComponentRelativePositionToParent(manipulator, my_name) + parent_position_to_world;
  my_orientation_to_world = parent_orientation_to_world * OM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, my_name), getComponentJointAngle(manipulator, my_name));

  setComponentPositionToWorld(manipulator, my_name, my_position_to_world);
  setComponentOrientationToWorld(manipulator, my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = getComponentChildName(manipulator, my_name).at(index);
    forward(manipulator, child_name);
  }
}

std::vector<float> Chain::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  // return positionOnlyInverseKinematics(manipulator, tool_name, target_pose);
  return srInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<float> Chain::inverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  const float lambda = 0.7;
  const int8_t iteration = 10;

  OM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));

  VectorXf pose_changed = VectorXf::Zero(6);
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));

  for (int8_t count = 0; count < iteration; count++)
  {
    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);

    jacobian = this->jacobian(&_manipulator, tool_name);

    pose_changed = OM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                           target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));
    if (pose_changed.norm() < 1E-6)
      return getAllActiveJointAngle(&_manipulator);

    ColPivHouseholderQR<MatrixXf> dec(jacobian);
    angle_changed = lambda * dec.solve(pose_changed);

    std::vector<float> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);
  }

  return getAllActiveJointAngle(&_manipulator);
}

std::vector<float> Chain::srInverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  float lambda = 0.0;
  const float param = 0.002;
  const int8_t iteration = 50;

  OM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));
  MatrixXf updated_jacobian = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));
  VectorXf pose_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf gerr(getDOF(&_manipulator));

  float wn_pos = 1 / 0.3;
  float wn_ang = 1 / (2 * M_PI);
  float Ek = 0.0;
  float Ek2 = 0.0;

  MatrixXf We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  MatrixXf Wn = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));

  forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
  pose_changed = OM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                         target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));
  Ek = pose_changed.transpose() * We * pose_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = Ek + param;

    updated_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = jacobian.transpose() * We * pose_changed;

    ColPivHouseholderQR<MatrixXf> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<float> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);

    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    pose_changed = OM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                           target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));

    Ek2 = pose_changed.transpose() * We * pose_changed;

    if (Ek2 < 1E-12)
    {
      return getAllActiveJointAngle(&_manipulator);
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<float> set_angle_changed;
      for (int8_t index = 0; index < getDOF(&_manipulator); index++)
        set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) - angle_changed(index));

      setAllActiveJointAngle(&_manipulator, set_angle_changed);

      forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    }
  }

  return getAllActiveJointAngle(&_manipulator);
}

std::vector<float> Chain::positionOnlyInverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  float lambda = 0.0;
  const float param = 0.002;
  const int8_t iteration = 10;

  OM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));
  MatrixXf position_jacobian = MatrixXf::Identity(3, getDOF(&_manipulator));
  MatrixXf updated_jacobian = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));
  VectorXf position_changed = VectorXf::Zero(3);
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf gerr(getDOF(&_manipulator));

  float wn_pos = 1 / 0.3;
  float wn_ang = 1 / (2 * M_PI);
  float Ek = 0.0;
  float Ek2 = 0.0;

  MatrixXf We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  MatrixXf Wn = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));

  forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
  position_changed = OM_MATH::positionDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name));
  Ek = position_changed.transpose() * We * position_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = Ek + param;

    updated_jacobian = (position_jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = position_jacobian.transpose() * We * position_changed;

    ColPivHouseholderQR<MatrixXf> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<float> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);

    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    position_changed = OM_MATH::positionDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name));

    Ek2 = position_changed.transpose() * We * position_changed;

    if (Ek2 < 1E-12)
    {
      return getAllActiveJointAngle(&_manipulator);
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<float> set_angle_changed;
      for (int8_t index = 0; index < getDOF(&_manipulator); index++)
        set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) - angle_changed(index));

      setAllActiveJointAngle(&_manipulator, set_angle_changed);

      forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    }
  }

  return getAllActiveJointAngle(&_manipulator);
}





// --------------- SCARA below --------------- //

MatrixXf SCARA::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
  chain_.jacobian(manipulator, tool_name);
}

void SCARA::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
  chain_.forward(manipulator, component_name);
}

void SCARA::forward(OM_MANAGER::Manipulator *manipulator)
{  
  chain_.forward(manipulator);
}

std::vector<float> SCARA::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

std::vector<float> SCARA::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<float> target_angle_vector;

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


// --------------- Link below --------------- //

MatrixXf Link::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Link::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

void Link::forward(OM_MANAGER::Manipulator *manipulator)
{
  Pose pose_to_wolrd;
  Pose link_relative_pose;
  Matrix3f rodrigues_rotation_matrix;
  Pose result_pose;

  //Base Pose Set (from world)
  pose_to_wolrd = getWorldPose(manipulator);
  link_relative_pose = getComponentRelativePoseToParent(manipulator, getWorldChildName(manipulator));
  rodrigues_rotation_matrix = OM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, getWorldChildName(manipulator)), getComponentJointAngle(manipulator, getWorldChildName(manipulator)));

  result_pose.position = pose_to_wolrd.position + pose_to_wolrd.orientation * link_relative_pose.position;
  result_pose.orientation = pose_to_wolrd.orientation * link_relative_pose.orientation * rodrigues_rotation_matrix;
  setComponentPoseToWorld(manipulator, getWorldChildName(manipulator), result_pose);

  //Next Component Pose Set
  for (int i = 0; i < getComponentChildName(manipulator, getWorldChildName(manipulator)).size(); i++)
  {
    solveKinematicsSinglePoint(manipulator, getComponentChildName(manipulator, getWorldChildName(manipulator)).at(i));
  }
}

std::vector<float> Link::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

void Link::solveKinematicsSinglePoint(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
  Pose parent_pose;
  Pose link_relative_pose;
  Matrix3f rodrigues_rotation_matrix;
  Pose result_pose;

  parent_pose = getComponentPoseToWorld(manipulator, getComponentParentName(manipulator, component_name));
  link_relative_pose = getComponentRelativePoseToParent(manipulator, component_name);
  rodrigues_rotation_matrix = OM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, component_name), getComponentJointAngle(manipulator, component_name));

  result_pose.position = parent_pose.position + parent_pose.orientation * link_relative_pose.position;
  result_pose.orientation = parent_pose.orientation * link_relative_pose.orientation * rodrigues_rotation_matrix;

  setComponentPoseToWorld(manipulator, component_name, result_pose);
  for (int i = 0; i < getComponentChildName(manipulator, component_name).size(); i++)
  {
    solveKinematicsSinglePoint(manipulator, getComponentChildName(manipulator, component_name).at(i));
  }
}

std::vector<float> Link::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose) //for basic model
{
  std::vector<float> target_angle_vector;
  Vector3f control_position; //joint6-joint1
  Vector3f tool_relative_position = getComponentRelativePositionToParent(manipulator, tool_name);
  Vector3f base_position = getComponentPositionToWorld(manipulator, getWorldChildName(manipulator));
  Vector3f temp_vector;

  float target_angle[3];
  float link[3];
  float temp_x;
  float temp_y;

  temp_x = target_pose.position(0) - base_position(0);
  temp_y = target_pose.position(1) - base_position(1);
  target_angle[0] = atan2(temp_y, temp_x);

  control_position(0) = target_pose.position(0) - tool_relative_position(0) * cos(target_angle[0]);
  control_position(1) = target_pose.position(1) - tool_relative_position(0) * sin(target_angle[0]);
  control_position(2) = target_pose.position(2) - tool_relative_position(2);

  // temp_vector = omlink.link_[0].getRelativeJointPosition(1,0);
  temp_vector = getComponentRelativePositionToParent(manipulator, getComponentParentName(manipulator, getComponentParentName(manipulator, getComponentParentName(manipulator, tool_name))));
  link[0] = temp_vector(2);
  // temp_vector = omlink.link_[1].getRelativeJointPosition(5,1);
  temp_vector = getComponentRelativePositionToParent(manipulator, getComponentParentName(manipulator, getComponentParentName(manipulator, tool_name)));
  link[1] = temp_vector(0);
  // temp_vector = omlink.link_[4].getRelativeJointPosition(6,5);
  temp_vector = getComponentRelativePositionToParent(manipulator, getComponentParentName(manipulator, tool_name));
  link[2] = temp_vector(0);

  temp_y = control_position(2) - base_position(2) - link[0];
  temp_x = (control_position(0) - base_position(0)) / cos(target_angle[0]);

  target_angle[1] = acos(((temp_x * temp_x + temp_y * temp_y + link[1] * link[1] - link[2] * link[2])) / (2 * link[1] * sqrt(temp_x * temp_x + temp_y * temp_y))) + atan2(temp_y, temp_x);
  target_angle[2] = acos((link[1] * link[1] + link[2] * link[2] - (temp_x * temp_x + temp_y * temp_y)) / (2 * link[1] * link[2])) + target_angle[1];

  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(-target_angle[1]);
  target_angle_vector.push_back(-target_angle[2]);

  return target_angle_vector;
}


// --------------- Planar below --------------- //

MatrixXf Planar::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Planar::forward(OM_MANAGER::Manipulator *manipulator)
{
}

void Planar::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

std::vector<float> Planar::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse2(manipulator, tool_name, target_pose);
}

// std::vector<float> Planar::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
// {
//   std::vector<float> target_angle_vector;

//   double temp_target_angle[3];
//   double target_angle[3];
//   double link[2];
//   double temp_x[3];     
//   double temp_y[3];
//   double target_pose_length[3];

//   // Link Lengths
//   link[0] = 0.120f;
//   link[1] = 0.098f;

//   // Pose for each set of two joints
//   temp_x[0] = target_pose.position(0);
//   temp_y[0] = target_pose.position(1);

//   temp_x[1] = cos(-PI*2.0f/3.0f)*temp_x[0] - sin(-PI*2.0f/3.0f)*temp_y[0];
//   temp_y[1] = sin(-PI*2.0f/3.0f)*temp_x[0] + cos(-PI*2.0f/3.0f)*temp_y[0];

//   temp_x[2] = cos(-PI*2.0f/3.0f)*temp_x[1] - sin(-PI*2.0f/3.0f)*temp_y[1];
//   temp_y[2] = sin(-PI*2.0f/3.0f)*temp_x[1] + cos(-PI*2.0f/3.0f)*temp_y[1];

//   // Length of Position Difference and Target Angle
//   target_pose_length[0] = sqrt(temp_y[0]*temp_y[0] + (temp_x[0]+0.1339)*(temp_x[0]+0.1339));
//   temp_target_angle[0] = acos((target_pose_length[0]*target_pose_length[0] + link[0]*link[0] - link[1]*link[1]) 
//                           / (2*target_pose_length[0]*link[0]));
//   target_angle[0] = acos(-temp_y[0] / target_pose_length[0]) - temp_target_angle[0] - PI/4;

//   target_pose_length[1] = sqrt(temp_y[1]*temp_y[1] + (temp_x[1]+0.1339)*(temp_x[1]+0.1339));
//   temp_target_angle[1] = acos((target_pose_length[1]*target_pose_length[1] + link[0]*link[0] - link[1]*link[1]) 
//                           / (2*ta
//   link[0] = 0.100f;
//   link[1] = 0.217f_pose_length[1]*link[0]));
//   target_angle[1] = acos(-temp_y[
//   link[0] = 0.100f;
//   link[1] = 0.217f target_pose_length[1]) - temp_target_angle[1] - PI/4;

//   target_pose_length[2] = sqrt(te
//   link[0] = 0.100f;
//   link[1] = 0.217f[2]*temp_y[2] + (temp_x[2]+0.1339)*(temp_x[2]+0.1339));
//   temp_target_angle[2] = acos((ta
//   link[0] = 0.100f;
//   link[1] = 0.217f_pose_length[2]*target_pose_length[2] + link[0]*link[0] - link[1]*link[1]) 
//                           / (2*ta
//   link[0] = 0.100f;
//   link[1] = 0.217f_pose_length[2]*link[0]));
//   target_angle[2] = acos(-temp_y[2] / target_pose_length[2]) - temp_target_angle[2] - PI/4;

//   // Set Joint Angle 
//   target_angle_vector.push_back(target_angle[0]);
//   target_angle_vector.push_back(target_angle[1]);
//   target_angle_vector.push_back(target_angle[2]);

//   return target_angle_vector;
// }

std::vector<float> Planar::geometricInverse2(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<float> target_angle_vector;

  float link[3];
  float start_x[3];    
  float start_y[3];
  float temp_x[3];       
  float temp_y[3];
  float goal_x[3];     
  float goal_y[3];
  Matrix3f goal_orientation;
  float diff_x[3];       
  float diff_y[3];
  float temp_target_angle[3];
  float temp_diff[3];      
  float target_pose_length[3];
  float target_angle[7];

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


// --------------- Delta below --------------- //

MatrixXf Delta::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Delta::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

void Delta::forward(OM_MANAGER::Manipulator *manipulator)
{
}

std::vector<float> Delta::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

std::vector<float> Delta::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<float> target_angle_vector;

  float temp_angle[3];
  float temp_angle2[3];
  float target_angle[3];
  float target_angle2[3];
  float target_angle3[3];
  float link[2];
  float start_x[3];       
  float start_y[3];
  float start_z[3];
  float target_x[3];       
  float target_y[3];
  float target_z[3];
  float diff_x[3];       
  float diff_y[3];
  float diff_z[3];
  float temp[3];
  float temp2[3];
  float target_pose_length[3];

  // Link Lengths
  link[0] = 0.100f;
  link[1] = 0.217f;

  // Start pose for each set of two joints
  for (int i=0; i<3; i++){
    start_x[i] = cos(PI*2.0/3.0*i)*(-0.055f);
    start_y[i] = sin(PI*2.0/3.0*i)*(-0.055f);
    start_z[i] = 0.169894;
  }
  
  // Goal pose for each set of two joints
  for (int i=0; i<3; i++){
    target_x[i] = target_pose.position(0) + cos(PI*2.0/3.0*i)*(-0.020f);
    target_y[i] = target_pose.position(1) + sin(PI*2.0/3.0*i)*(-0.020f);
    target_z[i] = target_pose.position(2);
  }

  // Pose difference for each set of two joints
  for (int i=0; i<3; i++){
    diff_x[i] = target_x[i] - start_x[i];
    diff_y[i] = target_y[i] - start_y[i];
    diff_z[i] = target_z[i] - start_z[i];
  }

  // Length of Position Difference and Target Angle
  // for (int i=0; i<3; i++){
  //   temp[i] = diff_x[i]*cos(PI*2.0/3.0*i)+diff_y[i]*sin(PI*2.0/3.0*i);
  //   temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);
  //   temp_angle[i] = acos((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
  //                   / (2.0*link[0]*temp2[i]));
  //   temp_angle2[i] = asin(diff_z[i] / temp2[i]);
  //   target_angle[i] = temp_angle[i] + temp_angle2[i];  
  // }

  for (int i=0; i<3; i++){
    temp[i] = diff_x[i]*cos(PI*2.0/3.0*i)+diff_y[i]*sin(PI*2.0/3.0*i);
    temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);
    temp_angle[i] = acos((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
                    / (2.0*link[0]*temp2[i]));
    temp_angle2[i] = acos(temp[i] / temp2[i]);
    target_angle[i] = +temp_angle[i] - temp_angle2[i];  
  }

  // Set Joint Angle 
  Serial.println(temp[0],4);
  Serial.println(temp2[0],4);
  Serial.println(diff_z[0],4);
  Serial.println(link[1]*link[1] - link[0]*link[0] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0] - diff_z[0]*diff_z[0], 4);
  Serial.println((link[1]*link[1] - link[0]*link[0] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0] - diff_z[0]*diff_z[0])/(-2.0*link[0]*temp2[0]),4);
  Serial.println(temp_angle[0],4);
  Serial.println(temp_angle2[0],4);
  Serial.println(target_angle[0],4);
  Serial.println(target_angle[1],4);
  Serial.println(target_angle[2],4);
  Serial.println(target_angle2[0],4);
  Serial.println(target_angle2[1],4);
  Serial.println(target_angle2[2],4);
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);



  // float felbow_x[3];       
  // float felbow_y[3];
  // float felbow_z[3];

  // float error = 1000000;
  // float ferroreach[3];
  // float temp_error;
  // float result[3];
  // for (int j=-70; j<70; j++){
  //   for (int k=-70; k<70; k++){
  //     for (int l=-30; l<30; l++){
  //       for (int i=0; i<3; i++){
  //         felbow_x[i] = cos(PI*2.0/3.0*i)*(0.033f) + link[0]*cos(target_angle2[i])*cos(PI*2.0/3.0*i);
  //         felbow_y[i] = sin(PI*2.0/3.0*i)*(0.033f) + link[0]*cos(target_angle2[i])*sin(PI*2.0/3.0*i);
  //         felbow_z[i] = -0.169894 + link[0]*sin(target_angle2[i]);
          
  //         ferroreach[i] = abs((felbow_x[i]+j/1000.0f)*(felbow_x[i]+j/1000.0f)
  //                      + (felbow_y[i]+k/1000.0f)*(felbow_y[i]+k/1000.0f) 
  //                      + (felbow_z[i]+l/1000.0f)*(felbow_z[i]+l/1000.0f) 
  //                      - link[1]*link[1]);
                       
  //       }
  //       temp_error = ferroreach[0] + ferroreach[1] + ferroreach[2];
  //       // Serial.println(temp_error);
        
  //       if (temp_error < error){
  //         error = temp_error;
  //         // Serial.println(error);
  //         // Serial.println(j);
  //         // Serial.println(k);
  //         // Serial.println(l);
  //         result[0] = j;
  //         result[1] = k;
  //         result[2] = l;
  //       }
  //     }
  //   }
  // }

  // Serial.println(result[0]);
  // Serial.println(result[1]);
  // Serial.println(result[2]);


  return target_angle_vector;
}







// --------------- Stewart below --------------- //

MatrixXf Stewart::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Stewart::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

void Stewart::forward(OM_MANAGER::Manipulator *manipulator)
{
}

std::vector<float> Stewart::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

std::vector<float> Stewart::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<float> target_angle_vector;

  float temp_angle[6];
  float temp_angle2[6];
  float target_angle[6];
  float link[2];
  float start_x[6];       
  float start_y[6];
  float start_z[6];
  float target_x[6];       
  float target_y[6];
  float target_z[6];
  float diff_x[6];       
  float diff_y[6];
  float diff_z[6];
  float temp[6];
  float temp2[6];
  float target_pose_length[6];

  // Link Lengths
  link[0] = 0.030f;    // modified the values
  link[1] = 0.110f;   // modified the values

  // Start pose for each set of two joints
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      start_x[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*(-0.080f);  // modified the values
      start_y[i] = sin(PI*2.0/3.0*(i/2) - 0.436)*(-0.080f); // modified the values
    } 
    else {
      start_x[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*(-0.080f);  // modified the values
      start_y[i] = sin(PI*2.0/3.0*(i/2) + 0.436)*(-0.080f); // modified the values
    }
    start_z[i] = -0.100; // modified the values
  }
  
  // Goal pose for each set of two joints
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      target_x[i] = target_pose.position(0) + cos(PI*2.0/3.0*(i/2) - 0.136)*(-0.07825f);
      target_y[i] = target_pose.position(1) + sin(PI*2.0/3.0*(i/2) - 0.136)*(-0.07825f);
    } 
    else {
      target_x[i] = target_pose.position(0) + cos(PI*2.0/3.0*(i/2) + 0.136)*(-0.07825f);
      target_y[i] = target_pose.position(1) + sin(PI*2.0/3.0*(i/2) + 0.136)*(-0.07825f);
    }
    target_z[i] = target_pose.position(2);
  }

  // Pose difference for each set of two joints
  for (int i=0; i<6; i++){
    diff_x[i] = target_x[i] - start_x[i];
    diff_y[i] = target_y[i] - start_y[i];
    diff_z[i] = target_z[i] - start_z[i];
  }
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      temp[i] = diff_x[i]*cos(PI*2.0/3.0*(i/2) - 0.436)+diff_y[i]*sin(PI*2.0/3.0*(i/2) - 0.436);  
    } 
    else {
      temp[i] = diff_x[i]*cos(PI*2.0/3.0*(i/2) + 0.436)+diff_y[i]*sin(PI*2.0/3.0*(i/2) + 0.436);  
    }
    temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);
    temp_angle[i] = acos((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
                    / (2.0*link[0]*temp2[i]));
    temp_angle2[i] = acos(temp[i] / temp2[i]);
    target_angle[i] = +temp_angle[i] - temp_angle2[i];  
  }

  // Set Joint Angle 
  Serial.println(temp[0],4);
  Serial.println(temp2[0],4);
  Serial.println(diff_z[0],4);
  Serial.println(link[1]*link[1] - link[0]*link[0] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0] - diff_z[0]*diff_z[0], 4);
  Serial.println((link[1]*link[1] - link[0]*link[0] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0] - diff_z[0]*diff_z[0])/(-2.0*link[0]*temp2[0]),4);
  Serial.println(temp_angle[0],4);
  Serial.println(temp_angle2[0],4);
  Serial.println(target_angle[0],4);
  Serial.println(-target_angle[1],4);
  Serial.println(target_angle[2],4);
  Serial.println(-target_angle[3],4);
  Serial.println(target_angle[4],4);
  Serial.println(-target_angle[5],4);
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






// --------------- Linear below --------------- //

MatrixXf Linear::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Linear::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

void Linear::forward(OM_MANAGER::Manipulator *manipulator)
{
}

std::vector<float> Linear::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

std::vector<float> Linear::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<float> target_angle_vector;

  float radius = 0.010;
  float x_length = 0.300;
  float y_length = 0.300;

  if (x_length > 0.300) x_length = 0.3;
  if (y_length > 0.300) y_length = 0.3;

  target_angle[0] = target_pose.position(0) / (2*radius);
  target_angle[1] = (x_length - target_pose.position(0)) / (2*radius);
  target_angle[2] = target_pose.position(1) / (2*radius);

  target_z[i] = target_pose.position(2);
  
  // Set Joint Angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  return target_angle_vector;
}

