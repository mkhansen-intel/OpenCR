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


#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "Stewart.h"
#include "demo.h"

void connectProcessing()
{
  stewart.connectProcessing(DXL_SIZE);
}

int availableProcessing()
{
  return Serial.available();
}

String readProcessingData()
{
  return Serial.readStringUntil('\n');
}

void fromProcessing(String data)
{
  String *cmd = stewart.parseDataFromProcessing(data);

  if (cmd[0] == "om")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      stewart.actuatorEnable();
      stewart.sendAngleToProcessing(stewart.receiveAllActuatorAngle());  
      stewart.sendToolData2Processing(stewart.getComponentToolValue(TOOL));
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      stewart.actuatorDisable();
#endif
    }
  }
  else if (cmd[0] == "joint")
  {
    std::vector<float> goal_position;
    
    for (uint8_t index = 0; index < ACTIVE_JOINT_SIZE; index++)
    {
      goal_position.push_back(cmd[index+1].toFloat());
    }

    stewart.jointMove(goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "f")
      stewart.setMove(TOOL, OM_MATH::makeVector3(0.030f, 0.000f, 0.0), 1.0);
    else if (cmd[1] == "b")
      stewart.setMove(TOOL, OM_MATH::makeVector3(-0.030f, 0.000f, 0.0), 1.0);
    else if (cmd[1] == "l")
      stewart.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.020f, 0.0), 1.0);
    else if (cmd[1] == "r")
      stewart.setMove(TOOL, OM_MATH::makeVector3(0.000f, -0.020f, 0.0), 1.0);
    else if (cmd[1] == "roti"){
      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(0.000f, 0.0, 0.000f);
      target_pose.orientation = OM_MATH::makeMatrix3(cos(PI/6.0),  -sin(PI/6.0),   0.0f,
                                                     sin(PI/6.0),   cos(PI/6.0),   0.0f,
                                                     0.0f,          0.0f,          1.0f);
      stewart.setPose(TOOL, target_pose, 1.0);
    }
    else if (cmd[1] == "rot"){
      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(0.000f, 0.0, 0.000f);
      target_pose.orientation = OM_MATH::makeMatrix3(cos(PI/4.0),  -sin(-PI/4.0),  0.0f,
                                                     sin(-PI/4.0),  cos(PI/4.0),   0.0f,
                                                     0.0f,          0.0f,          1.0f);
      stewart.setPose(TOOL, target_pose, 1.0);
    }
    else
      stewart.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.000f, 0.0), 1.0f);
  }
  else if (cmd[0] == "pos")
  {
    // Pose goal_pose;
    // goal_pose.position(0) = cmd[1].toFloat();
    // goal_pose.position(1) = cmd[2].toFloat();
    // goal_pose.position(2) = 0.0f;
    // goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);
    // planar.setPose(TOOL, goal_pose, 5.0f);
    
    if (~stewart.moving() && ~stewart.drawing())
    {
      // Serial.println(planar.moving());
      // Serial.println(planar.drawing());
      Vector3f pos;
      pos(0) = cmd[1].toFloat();
      pos(1) = cmd[2].toFloat();
      pos(2) = 0.0;
      stewart.drawLine2(TOOL, pos, 1.5); 
    }
  }
  else if (cmd[0] == "torque")
  {
#ifdef PLATFORM
    if (cmd[1] == "on")
      stewart.actuatorEnable();
    else if (cmd[1] == "off")
      stewart.actuatorDisable();
#endif
  }
  // else if (cmd[0] == "motion")
  // {
  //   if (cmd[1] == "start")
  //     test2();
  //   else if (cmd[1] == "stop")
  //     test2Stop();
  // }
}
#endif