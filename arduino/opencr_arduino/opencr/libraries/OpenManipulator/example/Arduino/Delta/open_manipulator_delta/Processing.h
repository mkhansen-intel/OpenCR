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

/* Authors: Darby Lim */

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "Delta.h"
#include "Suction.h"

// #include "Suction.h"

void connectProcessing()
{
  delta.connectProcessing(DXL_SIZE);
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
  String *cmd = delta.parseDataFromProcessing(data);

  if (cmd[0] == "om")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      delta.actuatorEnable();
      delta.sendAngleToProcessing(delta.receiveAllActuatorAngle());  
      delta.sendToolData2Processing(delta.getComponentToolValue(TOOL));
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      delta.actuatorDisable();
#endif
    }
  }
  else if (cmd[0] == "joint")
  {
    std::vector<float> goal_position;
    
    for (uint8_t index = 0; index < delta.getDOF(); index++)
    {
      goal_position.push_back(cmd[index+1].toFloat());
    }

    delta.jointMove(goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "forward")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.030f, 0.000f, 0.0), 1.0);
    else if (cmd[1] == "backward")
      delta.setMove(TOOL, OM_MATH::makeVector3(-0.030f, 0.000f, 0.0), 1.0);
    else if (cmd[1] == "left")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.030f, 0.0), 1.0);
    else if (cmd[1] == "right")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, -0.030f, 0.0), 1.0);
    else if (cmd[1] == "upward")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.0, 0.020f), 1.0);
    else if (cmd[1] == "downward")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.0, -0.020f), 1.0);
    else if (cmd[1] == "1")
      delta.setMove(TOOL, OM_MATH::makeVector3(-0.015f, -0.02598f, 0.0f), 1.0);
    else if (cmd[1] == "2")
      delta.setMove(TOOL, OM_MATH::makeVector3(-0.015f, 0.02598f, 0.0f), 1.0);
    else if (cmd[1] == "3")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.030f,  0.0, 0.0f), 1.0);
    else if (cmd[1] == "4")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.060f, 0.000f, 0.020f), 1.0);
    else if (cmd[1] == "5")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.060f, 0.020f), 1.0);
    else if (cmd[1] == "6")
      delta.setMove(TOOL, OM_MATH::makeVector3(-0.060f, 0.000f, 0.020f), 1.0);
    else
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.000f, 0.0), 1.0);
  }
  else if (cmd[0] == "pos")
  {
    // Pose goal_pose;
    // goal_pose.position(0) = cmd[1].toFloat();
    // goal_pose.position(1) = cmd[2].toFloat();
    // goal_pose.position(2) = 0.0f;
    // goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);
    // planar.setPose(TOOL, goal_pose, 5.0f);
    Pose goal_pose;
    goal_pose.position(0) = cmd[1].toFloat();
    goal_pose.position(1) = cmd[2].toFloat();
    goal_pose.position(2) = cmd[3].toFloat();
    goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);

    delta.setPose(TOOL, goal_pose, 1.5f);
  }
  // else if (cmd[0] == "suction")
  // {
  //   if (cmd[1] == "on")
  //   {
  //     suctionOn();
  //   }
  //   else if (cmd[1] == "off")
  //   {
  //     suctionOff();
  //   }
  // }
  else if (cmd[0] == "torque")
  {
#ifdef PLATFORM
    if (cmd[1] == "on")
      delta.actuatorEnable();
    else if (cmd[1] == "off")
      delta.actuatorDisable();
#endif
  }
}
#endif