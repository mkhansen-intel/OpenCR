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

#include <om_chain_lib.h>

String global_cmd[50];

void connectProcessing()
{ 
  for (int i = 0; i < DXL_SIZE; i++)
  {
    Serial.print(0.0);
    Serial.print(",");
  }

  Serial.println(0.0);
  delay(300);

  Serial.println("Init Processing");
}

int availableProcessing()
{
  return Serial.available();
}

String readProcessingData()
{
  return Serial.readStringUntil('\n');
}

void split(String data, char separator, String* temp)
{
  int cnt = 0;
  int get_index = 0;

  String copy = data;
  
  while(true)
  {
    get_index = copy.indexOf(separator);

	if(-1 != get_index)
	{
	  temp[cnt] = copy.substring(0, get_index);
  	  copy = copy.substring(get_index + 1);
	}
	else
	{
      temp[cnt] = copy.substring(0, copy.length());
	  break;
	}
	  ++cnt;
  }
}

String* parseDataFromProcessing(String get)
{
  get.trim();
  split(get, ',', global_cmd);
  
  return global_cmd;
}


void sendAngle2Processing(std::vector<double> joint_angle_vector)
{
  Serial.print("angle");

  for (int i = 0; i < joint_angle_vector.size(); i++)
  {
    Serial.print(",");
    Serial.print(joint_angle_vector.at(i));
  }
  Serial.print("\n");
}

void sendToolData2Processing(bool onoff)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(onoff);
  Serial.print("\n");
}

void sendToolData2Processing(double value)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(value);
  Serial.print("\n");
}

void sendValueToProcessing(OM_CHAIN *chain_)
{
  sendAngle2Processing(chain_->getManipulator()->getAllActiveJointValue());
  sendToolData2Processing(chain_->getManipulator()->getToolGoalValue(TOOL));
}


void fromProcessing(OM_CHAIN *chain_, String data)
{
  String *cmd = parseDataFromProcessing(data);

  if (cmd[0] == "opm")
  {
    if (cmd[1] == "ready")
    {
      if(chain_->getPlatformFlag())
      {
        chain_->allActuatorEnable();
        sendValueToProcessing(chain_);
      }
    }
    else if (cmd[1] == "end")
    {
      if(chain_->getPlatformFlag())
      {
        chain_->allActuatorDisable();
      }
    }
  }
  else if (cmd[0] == "joint")
  {
    std::vector<double> goal_position;

    for (uint8_t index = 0; index < DXL_SIZE; index++)
    {
      goal_position.push_back((double)cmd[index + 1].toFloat());
    }

    chain_->jointTrajectoryMove(goal_position, 1.0); // FIX TIME PARAM
  }
  else if (cmd[0] == "gripper")
  {
    chain_->toolMove(TOOL, (double)cmd[1].toFloat());
  }
  else if (cmd[0] == "grip")
  {
    if (cmd[1] == "on")
      chain_->toolMove(TOOL, 0.0);
    else if (cmd[1] == "off")
      chain_->toolMove(TOOL, -1.0);
  }
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "forward")
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.010, 0.0, 0.0), 0.2);
    else if (cmd[1] == "back")
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(-0.010, 0.0, 0.0), 0.2);
    else if (cmd[1] == "left")
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.010, 0.0), 0.2);
    else if (cmd[1] == "right")
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, -0.010, 0.0), 0.2);
    else if (cmd[1] == "up")
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.0, 0.010), 0.2);
    else if (cmd[1] == "down")
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.0, -0.010), 0.2);
    else
      chain_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.0, 0.0), 0.2);
  }
  else if (cmd[0] == "torque")
  {
    if(chain_->getPlatformFlag())
    {
      if (cmd[1] == "on")
        chain_->allActuatorEnable();
      else if (cmd[1] == "off")
        chain_->allActuatorDisable();
    }
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      Pose present_pose = chain_->getManipulator()->getComponentPoseToWorld(TOOL);
      WayPoint draw_goal_pose[6];
      draw_goal_pose[0].value = present_pose.position(0) + 0.02;
      draw_goal_pose[1].value = present_pose.position(1) + 0.02;
      draw_goal_pose[2].value = present_pose.position(2) - 0.02;
      draw_goal_pose[3].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[0];
      draw_goal_pose[4].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[1];
      draw_goal_pose[5].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[2];

      void *p_draw_goal_pose = &draw_goal_pose;
      
      chain_->drawingTrajectoryMove(DRAWING_LINE, TOOL, p_draw_goal_pose, 1.0);
    }
    else if (cmd[1] == "stop")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = 0.03; // radius (m)
      draw_circle_arg[1] = 2;    // revolution
      draw_circle_arg[2] = 0.0;  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      chain_->drawingTrajectoryMove(DRAWING_CIRCLE, TOOL, p_draw_circle_arg, 4.0);

    }
  }
}
#endif
