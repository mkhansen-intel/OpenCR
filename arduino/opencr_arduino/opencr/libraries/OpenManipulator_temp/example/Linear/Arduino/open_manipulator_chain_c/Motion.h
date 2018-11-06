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

/* Authors: Hye-Jong KIM */

#ifndef MOTION_H_
#define MOTION_H_

#include "Chain.h"
#include <unistd.h>


using namespace OPEN_MANIPULATOR;


bool motion[2]    = {false, false};
bool continue_flag = false;
uint8_t motion_cnt[2] = {0,0};
uint8_t sub_motion_cnt[2] = {0, 0};

void gripOnOff(OpenManipulator* manipulator, bool data)
{
  if(data)
    manipulator->toolMove(TOOL, -0.9f); // gripper open
  else
    manipulator->toolMove(TOOL, 0.6f); // gripper close
}
// Move in Joint Space
void moveJS(OpenManipulator* manipulator, float j1, float j2, float j3, float j4, float t)
{
  static std::vector <float> target_angle;
  target_angle.clear();
  target_angle.push_back(j1);
  target_angle.push_back(j2);
  target_angle.push_back(j3);
  target_angle.push_back(j4);
  manipulator->jointMove(target_angle,t);     
}
// Move in Cartesian Space
void moveCS(OpenManipulator* manipulator, float x, float y, float z, float t)
{
  Pose target_pose = manipulator->getComponentPoseToWorld(TOOL);
  target_pose.position(0) = x;
  target_pose.position(1) = y;
  target_pose.position(2) = z;
  manipulator->setPose(TOOL, target_pose, t);
}

void motionStart()
{
  // For Robot1
  motion_cnt[0] = 0;          
  sub_motion_cnt[0] = 0;
  motion[0] = true;

  // For Robot2
  motion_cnt[1] = 0;    
  sub_motion_cnt[1] = 0;
  motion[1] = true;

  continue_flag = false;
}

void motionStop()
{
  motion_cnt[0] = 0;          
  sub_motion_cnt[0] = 0;
  motion[0] = false;
}

bool motion7(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.0, -1.5, -0.0,   1.5, 2.0); break;
  case 1: moveJS(manipulator,0.44,-0.00, 1.00, -1.00, 2.0); break;
  case 2: moveJS(manipulator,0.44, 0.10,  0.8, -0.90, 5.0); break;
  case 3: moveJS(manipulator,0.44, 0.10,  0.8, -0.90, 2.0); break;
  case 4: moveJS(manipulator,0.44,-0.00, 1.00, -1.00, 5.0); break;
  // case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.100, 0.0, 0.0f), 0.5f); return 1; 
  }
  return 0;
}
bool motion8(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.0, -1.5,  -0.0,   1.5, 2.0); break;
  case 1: moveJS(manipulator,0.8,-1.00,  1.00, -1.00, 2.0); break;
  case 2: moveJS(manipulator, 0.0, -1.5,  -0.0,   1.5, 2.0); break;
  case 3: moveJS(manipulator,-0.8,-1.00, 1.00, -1.00, 2.0); break;
  case 4: moveJS(manipulator, 0.0, -1.5,  -0.0,   1.5, 2.0); break;
  case 5: moveJS(manipulator,0.8,-1.00,  1.00, -1.00, 2.0); break;
  case 6: moveJS(manipulator, 0.0, -1.5,  -0.0,   1.5, 2.0); break;
  case 7: moveJS(manipulator,-0.8,-1.00, 1.00, -1.00, 2.0); break;
  case 8: moveJS(manipulator, 0.0, -1.5,  -0.0,   1.5, 2.0); break;
  // case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.100, 0.0, 0.0f), 0.5f); return 1; 
  }
  return 0;
}


// --------------- Set Motion for OM1 --------------- //
void setMotion1()
{
  Serial.println(motion[0]);
  Serial.println(!chain.checkManipulatorMoving());
  gripOnOff(&chain, false);

  if(motion[0] && !chain.checkManipulatorMoving())
  {
////////////////////////MOTION SETTING//////////////////////////////        

    switch(motion_cnt[0])
    {
    ////////// O X O 
    case 0:
      if(motion8(&chain, 0))
      { sub_motion_cnt[0] = 0;  }
// motion_cnt[0] ++;
      else 
        sub_motion_cnt[0] ++;
    break;
    }
////////////////////////MOTION SETTING////////////////////////////// 
  }
}

#endif
