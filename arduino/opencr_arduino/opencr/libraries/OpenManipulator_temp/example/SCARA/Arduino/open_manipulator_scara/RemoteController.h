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

#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#include <SCARA.h>
#include <RC100.h>

RC100 rc100;
double grip_value = 0.0;

void connectRC100()
{
  rc100.begin(1);
}

int availableRC100()
{
  return rc100.available();
}

uint16_t readRC100Data()
{
  return rc100.readData();
}

void fromRC100(SCARA* SCARA_, uint16_t data)
{
  if (data & RC100_BTN_U)
    SCARA_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.007, 0.0, 0.0), 0.16);
  else if (data & RC100_BTN_D)
    SCARA_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(-0.007, 0.0, 0.0), 0.16);
  else if (data & RC100_BTN_L)
    SCARA_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.007, 0.0), 0.16);
  else if (data & RC100_BTN_R)
    SCARA_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, -0.007, 0.0), 0.16);
  else if (data & RC100_BTN_1)
    SCARA_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.0, 0.007), 0.16);
  else if (data & RC100_BTN_3)
    SCARA_->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(0.0, 0.0, -0.007), 0.16);
  else if (data & RC100_BTN_2)
  {
    grip_value += 0.030;
    if (grip_value >= 0.907f)
      grip_value = 0.907f;

    SCARA_->toolMove(TOOL, grip_value);
  }
  else if (data & RC100_BTN_4)
  {
    grip_value -= 0.030;
    if (grip_value <= -1.130f)
      grip_value = -1.130f;

    SCARA_->toolMove(TOOL, grip_value);
  }
  else if (data & RC100_BTN_5)
  {
    std::vector<double> goal_position;

    goal_position.push_back(0.0);
    goal_position.push_back(-60.0 * DEG2RAD);
    goal_position.push_back(20.0 * DEG2RAD);
    goal_position.push_back(40.0 * DEG2RAD);

    SCARA_->jointTrajectoryMove(goal_position, 1.5);
  }
  else if (data & RC100_BTN_6)
  {
    std::vector<double> goal_position;

    goal_position.push_back(0.0);
    goal_position.push_back(0.0);
    goal_position.push_back(0.0);
    goal_position.push_back(0.0);

    SCARA_->jointTrajectoryMove(goal_position, 1.0);
  }
}
#endif
