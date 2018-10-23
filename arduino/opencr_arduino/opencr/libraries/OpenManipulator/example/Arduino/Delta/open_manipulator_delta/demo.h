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

#ifndef DEMO_H_
#define DEMO_H_

#include "Delta.h"

#define MAX_MOTION_NUM 12
#define MAX_MOTION_NUM2 27


bool suction_on = false;

const float tool_position[MAX_MOTION_NUM][3] = {// { x, y, z}
                                                {-0.060f,  0.000f, 0.030f},  
                                                {-0.060f,  0.000f, 0.020f},  
                                                {-0.060f,  0.000f, 0.030f},  
                                                { 0.000f,  0.060f, 0.030f},  
                                                { 0.000f,  0.060f, 0.020f},  
                                                { 0.000f,  0.060f, 0.030f},  
                                                { 0.060f,  0.000f, 0.030f},  
                                                { 0.060f,  0.000f, 0.020f},  
                                                { 0.060f,  0.000f, 0.030f},  
                                                {-0.000f, -0.060f, 0.030f},  
                                                {-0.000f, -0.060f, 0.020f},  
                                                {-0.000f, -0.060f, 0.030f}
                                                // {-0.050f, 0.000f, -0.150f},  
                                                // {0.000f,  0.000f, -0.150f},  
                                                // {0.000f, -0.050f, -0.150f},  
                                                // {0.000f,  0.000f, -0.150f},  
                                                };
const float tool_position2[MAX_MOTION_NUM2][3] = {// { x, y, z}
                                                { 0.0563f, -0.0325f, 0.030f},  
                                                { 0.0563f, -0.0325f, 0.020f},  
                                                { 0.0563f, -0.0325f, 0.030f},  
                                                { 0.000f,  -0.065f,  0.030f},  
                                                { 0.000f,  -0.065f,  0.020f},  
                                                { 0.000f,  -0.065f,  0.030f},  
                                                {-0.0563f, -0.0325f, 0.030f},  
                                                {-0.0563f, -0.0325f, 0.020f},  
                                                {-0.0563f, -0.0325f, 0.030f},  
                                                {-0.0563f,  0.0325f, 0.030f},  
                                                {-0.0563f,  0.0325f, 0.020f},  
                                                {-0.0563f,  0.0325f, 0.030f},  
                                                { 0.000f,   0.065f,  0.030f},  
                                                { 0.000f,   0.065f,  0.020f},  
                                                { 0.000f,   0.065f,  0.030f},  
                                                { 0.0563f,  0.0325f, 0.030f},  
                                                { 0.0563f,  0.0325f, 0.020f},  
                                                { 0.0563f,  0.0325f, 0.030f},  
                                                { 0.000f,   0.0173f, 0.030f},  
                                                { 0.000f,   0.0173f, 0.020f},  
                                                { 0.000f,   0.0173f, 0.030f},  
                                                {-0.0150f, -0.00865f,0.030f},  
                                                {-0.0150f, -0.00865f,0.020f},  
                                                {-0.0150f, -0.00865f,0.030f},  
                                                { 0.0150f, -0.00865f,0.030f},  
                                                { 0.0150f, -0.00865f,0.020f},  
                                                { 0.0150f, -0.00865f,0.030f}
                                                };

uint8_t motion_cnt = 0;

void test()
{
  if (delta.moving())
  {
    return;
  }
  else
  {
    if (motion_cnt == MAX_MOTION_NUM)
      motion_cnt = 0;

    // delta.setMove(TOOL, OM_MATH::makeVector3(
    //     tool_position[motion_cnt][0] / 1.0, 
    //     tool_position[motion_cnt][1] / 1.0, 
    //     tool_position[motion_cnt][2] / 1.0), 
    //     0.4);
    // motion_cnt++;
  }
}


void test2()
{
  if (delta.moving())
  {
    return;
  }
  else
  {
    if (motion_cnt == MAX_MOTION_NUM2)
      motion_cnt = 0;

    // if (~suction_on){
      // digitalWrite(RELAY_PIN, HIGH);      //suction on
    //   suction_on = true;
    // } 
    // else{
      // digitalWrite(RELAY_PIN, LOW);      //suction off
      // suction_on = false;
    // }
    delta.setMove(TOOL, OM_MATH::makeVector3(
        tool_position2[motion_cnt][0] / 1.0, 
        tool_position2[motion_cnt][1] / 1.0, 
        tool_position2[motion_cnt][2] / 1.0), 
        0.2);
    motion_cnt++;
  }
}

float start_angular_position = 0.0f;
const float move_time = 10.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.020f;
void test3()
{
  if (delta.moving() || delta.drawing())
  {
    return;
  }
  else
  {
    radius = 0.010f;
    delta.drawInit(SPIRAL, move_time, p_init_arg);
    delta.setRadiusForDrawing(SPIRAL, radius);
    delta.setStartPositionForDrawing(SPIRAL, delta.getComponentPositionToWorld(TOOL));
    delta.setStartAngularPositionForDrawing(SPIRAL, start_angular_position);
    delta.draw(SPIRAL);
  }
}

void test4()
{
  if (delta.moving())
  {
    return;
  }
  else
  {
    if (motion_cnt == MAX_MOTION_NUM)
      motion_cnt = 0;

    Pose target_pose;
    target_pose.position = OM_MATH::makeVector3(tool_position[motion_cnt][0], 
                                                tool_position[motion_cnt][1], 
                                                tool_position[motion_cnt][2]);
    target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                   0.0f, 1.0f, 0.0f,
                                                   0.0f, 0.0f, 1.0f);

    delta.setPose(TOOL, target_pose, 0.4);
    motion_cnt++;
  }
}


#endif // DEMO_H_
