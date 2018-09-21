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
      delta.setMove(TOOL, OM_MATH::makeVector3(0.020f, 0.000f, 0.0), 1.0);
    else if (cmd[1] == "backward")
      delta.setMove(TOOL, OM_MATH::makeVector3(-0.020f, 0.000f, 0.0), 1.0);
    else if (cmd[1] == "left")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.020f, 0.0), 1.0);
    else if (cmd[1] == "right")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, -0.020f, 0.0), 1.0);
    else if (cmd[1] == "upward")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.0, 0.020f), 1.0);
    else if (cmd[1] == "downward")
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.0, -0.020f), 1.0);
    else
      delta.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.000f, 0.0), 1.0);
  }
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