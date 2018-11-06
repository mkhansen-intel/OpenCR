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

#define CIRCLE 11
#define RHOMBUS 13
#define HEART 15
#define BOTTLESHAKE 17
#define BOTTLESHAKEY 18
#define BOTTLESHAKEX 19
#define BOTTLESHAKE3 20

using namespace OPEN_MANIPULATOR;

OPEN_MANIPULATOR::Draw *circle = new OM_PATH::Circle();
OPEN_MANIPULATOR::Draw *rhombus = new OM_PATH::Rhombus();
OPEN_MANIPULATOR::Draw *heart = new OM_PATH::Heart();
OPEN_MANIPULATOR::Draw *bottleshake = new OM_PATH::BottleShake();
OPEN_MANIPULATOR::Draw *bottleshakeY = new OM_PATH::BottleShakeY();
OPEN_MANIPULATOR::Draw *bottleshakeX = new OM_PATH::BottleShakeX();
OPEN_MANIPULATOR::Draw *bottleshake3 = new OM_PATH::BottleShake3();

OPEN_MANIPULATOR::Draw *circle2 = new OM_PATH::Circle();
OPEN_MANIPULATOR::Draw *rhombus2 = new OM_PATH::Rhombus();
OPEN_MANIPULATOR::Draw *heart2 = new OM_PATH::Heart();
OPEN_MANIPULATOR::Draw *bottleshake2 = new OM_PATH::BottleShake();
OPEN_MANIPULATOR::Draw *bottleshakeY2 = new OM_PATH::BottleShakeY();
OPEN_MANIPULATOR::Draw *bottleshakeX2 = new OM_PATH::BottleShakeX();
OPEN_MANIPULATOR::Draw *bottleshake32 = new OM_PATH::BottleShake3();

bool motion[2]    = {false, false};
bool continue_flag = false;
uint8_t motion_cnt[2] = {0,0};
uint8_t sub_motion_cnt[2] = {0, 0};

void gripOnOff(OpenManipulator* manipulator, bool data)
{
  if(data)
    manipulator->toolMove(TOOL, -0.9f); // gripper open
  else
    manipulator->toolMove(TOOL, 0.15f); // gripper close
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
  // For OM1
  motion_cnt[0] = 0;          
  sub_motion_cnt[0] = 0;
  motion[0] = false;

  // For OM2
  motion_cnt[1] = 0;          
  sub_motion_cnt[1] = 0;
  motion[1] = false;
}

bool pickStickMiddle(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: gripOnOff(manipulator, true); moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.0);  break;
  case 1: /*OM2 가운데 구멍 위치 (x, y, z, time)*/
    if(index == 0) moveCS(manipulator, -0.038, -0.000, 0.15, 1.5);     
    else if(index == 1) moveCS(manipulator, -0.040, -0.000, 0.15, 1.5); 
    break;
  case 2: // 집으러 내려가기
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.060f), 1.0f);      
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.073f), 1.0f);
    break;
  case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.115f), 0.6f); return 1; 
  }
  return 0;
}
bool pickStickLeft(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0:  gripOnOff(manipulator, true); moveJS(manipulator, -0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: /*OM2 왼쪽 구멍 위치 (x, y, z, time)*/
    if(index == 0) moveCS(manipulator, -0.040, -0.072, 0.15, 1.5); 
    else if(index == 1) moveCS(manipulator, -0.040, -0.080, 0.15, 1.5); 
    break;
  case 2: // 집으러 내려가기
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.060f), 1.0f);      
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.073f), 1.0f);
    break;
  case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.01, 0.0, 0.115f), 0.6f);  // x 앞으로 살짝 밀며 꺼내기
          return 1; 
  }
  return 0;
}
bool pickStickRight(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: gripOnOff(manipulator, true); moveJS(manipulator, 0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: /*OM2 오른쪽 구멍 위치 (x, y, z, time)*/
    if(index == 0) moveCS(manipulator, -0.044, 0.079, 0.15, 1.5);
    else if(index == 1) moveCS(manipulator, -0.040, 0.080, 0.15, 1.5); 
    break;
  case 2: // 집으러 내려가기
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.060f), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.073f), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.01, 0.0, 0.115f), 0.6f); // x 앞으로 살짝 밀며 꺼내기
          return 1; 
  }
  return 0;
}

bool placeStickMiddle(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.115f;
  if(opt == 1)
    depth = -0.031f; // 전달 받았을때 놓기위해 내려가는 z 높이
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.043, -0.000, 0.200, 1.5); 
    else if(index == 1) moveCS(manipulator, -0.040, -0.000, 0.19, 1.5); /*OM2 가운데 구멍 위치 (x, y, z, time)*/
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; 
  }
  return 0;
}
bool placeStickLeft(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.140f;
  if(opt == 1)
    depth = -0.070f; 
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, -0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.038, -0.072, 0.20, 1.5); 
    else if(index == 1) moveCS(manipulator, -0.044, -0.080, 0.22, 1.5); /*OM2 왼쪽 구멍 위치 (x, y, z, time)*/
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; 
  }
  return 0;
}
bool placeStickRight(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.140f;
  if(opt == 1)  // 1 == 전달받음 / 0 == 전달 안받음
    depth = -0.070f; // 전달 받았을때 놓기위해 내려가는 z 높이

  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.044, 0.079, 0.22, 1.5);
    else if(index == 1) moveCS(manipulator, -0.040, 0.080, 0.20, 1.5); /*OM2 오른쪽 구멍 위치 (x, y, z, time)*/
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth-0.013), 1.0f); // 전달 받았을때 놓기위해 내려가는 z 높이
    break;
  case 3: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; 
  }
  return 0;
}

bool motion1(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.00, -1.05, 0.35,   0.7, 2.0);  break;
  case 1: moveJS(manipulator, -2.00, -1.22, 0.75, -0.76, 2.0); break;
  case 2: moveJS(manipulator, -0.52, -1.20, 0.05,  1.63, 2.0); break;
  case 3: moveJS(manipulator,  0.52, -1.22, 0.75, -0.76, 2.0); break;
  case 4: moveJS(manipulator,  2.00, -1.20, 0.05,  1.63, 2.0); return 1; 
  }
  return 0;
}
bool motion2(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.00, -1.05, 0.35,   0.7, 2.0);  break;
  case 1: moveJS(manipulator,  2.00, -1.20, 0.05,  1.63, 2.0); break;
  case 2: moveJS(manipulator,  0.52, -1.22, 0.75, -0.76, 2.0); break;
  case 3: moveJS(manipulator, -0.52, -1.20, 0.05,  1.63, 2.0); break;
  case 4: moveJS(manipulator, -2.00, -1.22, 0.75, -0.76, 2.0); return 1; 
  }
  return 0;
}

bool send1to2(OpenManipulator* manipulator, int index)
{
  if(index == 0)
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); break; // init
    case 1: moveJS(manipulator, 1.57, -1.00, 1.44, -0.47, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.05f), 1.0f); break;
    case 3: manipulator->wait(1.5); break;
    case 4: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, 1.57, -1.00, 1.44, -0.47, 1.5);   return 1; 
    }
  }
  else if(index == 1)
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); gripOnOff(manipulator, true); break; // init
    case 1: moveJS(manipulator, -1.53, -0.71, -0.14, 0.82, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.09f), 1.0f); break;
    case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
    case 4: manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, -1.53, -0.71, -0.14, 0.82, 1.5); return 1; 
    }
  }
  return 0;
}

bool send2to1(OpenManipulator* manipulator, int index)
{
  if(index == 0)  //recv
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); gripOnOff(manipulator, true); break; // init
    case 1: moveJS(manipulator, 1.57, -0.71, -0.14, 0.82, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.09f), 1.0f); break;
    case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
    case 4: manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, 1.57,  -0.71, -0.14, 0.82, 1.5); return 1; 
    }
  }
  else if(index == 1) //send
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); break; // init
    case 1: moveJS(manipulator, -1.53, -1.00, 1.44, -0.47, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.05f), 1.0f); break;
    case 3: manipulator->wait(1.5); break;
    case 4: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, -1.53, -1.00, 1.44, -0.47, 1.5); return 1; 
    }
  }
  
  return 0;
}

bool motion_place_6_7(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, -0.70, -0.61, 0.80,  -0.13, 2.0); gripOnOff(manipulator, false); break;
  case 1: moveJS(manipulator, -0.35,  0.01, 1.28,  -1.28, 1.6); break;
  case 2: moveJS(manipulator,  0.35,  0.01, 1.28,  -1.28, 1.7); break;
  case 3: moveJS(manipulator,  0.70, -0.61, 0.80,  -0.13, 1.7); break;
  case 4: moveJS(manipulator,  0.00, -0.97, 0.03,   0.96, 1.6); return 1; 
  }
  return 0;
}

bool motion_pick_6_6(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.00, -1.05,  0.35, 0.70, 2.0); gripOnOff(manipulator, false); break;
  case 1: moveJS(manipulator, -0.80, -0.95,  0.53, 0.42, 1.6); break;
  case 2: moveJS(manipulator,  0.00, -1.53,  1.40, 0.13, 1.7); break;
  case 3: moveJS(manipulator,  0.80, -0.95,  0.53, 0.42, 1.7); break;
  case 4: moveJS(manipulator,  0.00, -0.60, -0.57, 1.17, 1.6); return 1; 
  }
  return 0;
}

// --------------- Draw Objects --------------- //
void drawObj(OpenManipulator* manipulator, float move_time, int object, float radius, float start_angular_position)
{
  float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
  void *p_init_arg = init_arg;
  manipulator->drawInit(object, move_time, p_init_arg);
  manipulator->setRadiusForDrawing(object, radius);
  manipulator->setStartPositionForDrawing(object, manipulator->getComponentPositionToWorld(TOOL));
  manipulator->setStartAngularPositionForDrawing(object, start_angular_position);
  manipulator->draw(object);
}

bool motion4(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.0, -0.86, 0.33, 0.53, 2.0);  break;
  case 1: drawObj(manipulator, 2.0, BOTTLESHAKE, 0.030, 0.0); return 1; 
  }
  return 0;
}

bool motion5_2(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.0, -0.86, 0.33, 0.53, 2.0); 
  // break;
  // case 1: drawObj(manipulator, 1.5, BOTTLESHAKEX, 0.035, 0.0); 
  return 1; 
  }
  return 0;
}
bool motion5_1(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.0, -0.56, 0.33, 0.53, 2.0); break;
  case 1: drawObj(manipulator, 1.5, BOTTLESHAKEY, 0.035, 0.0); 
  return 1; 
  }
  return 0;
}
bool motion5_3(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.0, -0.86, 0.33, 0.53, 2.0); break;
  case 1: drawObj(manipulator, 1.5, BOTTLESHAKEX, 0.035, 0.0); return 1; 
  }
  return 0;
}

bool motion6(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.0, -0.86, 0.33, 0.53, 2.0); break;
  case 1: drawObj(manipulator, 1.5, BOTTLESHAKE3, 0.03, 0.0); return 1; 
  }
  return 0;
}

bool motion7(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.140f;
  if(opt == 1)  // 1 == 전달받음 / 0 == 전달 안받음
    depth = -0.070f; // 전달 받았을때 놓기위해 내려가는 z 높이
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 0.5f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 0.5f); 
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -depth), 0.5f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -depth), 0.5f); 
    break;
  case 3: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(depth, 0.0, 0.0), 0.5f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(depth, 0.0, 0.0), 0.5f); 
    break;
  case 4: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(-depth, 0.0, 0.0), 0.5f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(-depth, 0.0, 0.0), 0.5f); 
    return 1; 
  }
  return 0;
}

// Set Motion for OM1
void setMotion1()
{
  if(motion[0] && !chain.checkManipulatorMoving())
  {
////////////////////////MOTION SETTING//////////////////////////////        
    switch(motion_cnt[0])
    {
    ////////// O X O 
    case 0:
      if(pickStickLeft(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 1:
      if(motion4(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 2:
      if(motion5_1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 3:
      if(motion5_2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 4:
      if(motion7(&chain, 0, 1))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    }
////////////////////////MOTION SETTING////////////////////////////// 
  }
}

// Set Motion for OM1
void setMotion2()
{
  if(motion[1] && !chain2.checkManipulatorMoving())
  {
////////////////////////MOTION SETTING//////////////////////////////        
    switch(motion_cnt[1])
    {
    ////////// O X O 
    case 0:
      if(pickStickRight(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 1:
      if(motion4(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 2:
      if(motion5_1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 3:
      if(motion5_2(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 4:
      if(motion7(&chain2, 1, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    }
////////////////////////MOTION SETTING////////////////////////////// 
  }
}

#endif

// --------------- Storyline --------------- //
//  	  OM1	        OM2
// 	    O X O	      O X O
// 0	  Pick 2	    Pick 3
// 1	  motion5_2   motion5_3
// 2	  motion5_1	  motion5_1
// 3	  Place 1	    Place 1
// 	    X O O	      O O X
// 4	  Pick 1	    Pick M
// 5	  send 1to2	  send 1to2
// 6	  Place M	    Place 3
// 	    X X O	      O O O
// 7	  Pick M	    Pick 1
// 8	  send 2to1	  send 2to1
// 9	  Place 2	    Place M
// 	    O X O	      O X O
// 10	  Pick 3	    Pick 2
// 11	  Motion 4	  Motion 4
// 12	  Place 1	    Place 1
// 	    O O X	      X O O
// 13	  Pick 2	    Pick 3
// 14	  motion5_2	  motion5_2
// 15	  motion5_1	  motion5_1
// 16	  motion5_3	  motion5_3
// 17	  Place 3	    Place 2
// 	    X O O	      O O X
// 18	  Pick M	    Pick 1
// 19	  send 2to1	  send 2to1
// 20	  Place 2	    Place M
// 	    O O O	      O X X
// 21	  Pick 1	    Pick M
// 22	  send 1to2	  send 1to2
// 23	  Place M	    Place 3
// 	    O X O	      O X O


// --------------- Unused functions --------------- //

// bool shakeMotion1(OpenManipulator* manipulator, int index)
// {
//   switch(sub_motion_cnt[index])
//   {
//   case 0: moveJS(manipulator, 0.0, -1.05, 0.35,  0.7,  2.0);  break;
//   case 1: moveJS(manipulator, 0.0, -0.92, 0.08, -0.64, 0.8); break;
//   case 2: moveJS(manipulator, 0.0, -1.35, 0.81,  0.34, 0.8); break;
//   case 3: moveJS(manipulator, 0.0, -0.92, 0.08, -0.64, 0.8); break;
//   case 4: moveJS(manipulator, 0.0, -1.35, 0.81,  0.34, 0.8); break;
//   case 5: moveJS(manipulator, 0.0, -0.92, 0.08, -0.64, 0.8); 
//           return 1; 
//   }
//   return 0;
// }
// bool shakeMotion3(OpenManipulator* manipulator, int index)
// {
//   switch(sub_motion_cnt[index])
//   {
//   case 0: moveJS(manipulator, 0,    -1.05, 0.35,  0.7,  2.0);  break;
//   case 1: moveJS(manipulator, 1.57, -0.92, 0.08, -0.64, 1.5); break;
//   case 2: moveJS(manipulator, 1.57, -1.35, 0.81,  0.34, 0.8); break;
//   case 3: moveJS(manipulator, 1.57, -0.92, 0.08, -0.64, 0.8); break;
//   case 4: moveJS(manipulator, 1.57, -1.35, 0.81,  0.34, 0.8); break;
//   case 5: moveJS(manipulator, 1.57, -0.92, 0.08, -0.64, 0.8); return 1; 
//   }
//   return 0;
// }
// bool shakeMotion2(OpenManipulator* manipulator, int index)
// {
//   switch(sub_motion_cnt[index])
//   {
//   case 0: moveJS(manipulator, 0,     -1.05, 0.35,  0.7,  2.0);  break;
//   case 1: moveJS(manipulator, -1.57, -0.92, 0.08, -0.64, 1.5); break;
//   case 2: moveJS(manipulator, -1.57, -1.35, 0.81,  0.34, 0.8); break;
//   case 3: moveJS(manipulator, -1.57, -0.92, 0.08, -0.64, 0.8); break;
//   case 4: moveJS(manipulator, -1.57, -1.35, 0.81,  0.34, 0.8); break;
//   case 5: moveJS(manipulator, -1.57, -0.92, 0.08, -0.64, 0.8); return 1; 
//   }
//   return 0;
// }