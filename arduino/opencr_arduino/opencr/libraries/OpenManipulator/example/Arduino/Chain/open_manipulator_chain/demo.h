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

#include "Chain.h"

// #define MAX_MOTION_NUM 12
// #define MAX_MOTION_NUM2 45
// #define MAX_MOTION_NUM3 60
// #define MAX_MOTION_NUM4 60


bool suction_on = false;

#define MAX_MOTION_NUM7 140
const float tool_position7[MAX_MOTION_NUM7][4] = {// { x, y, z}
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  

                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                {-0.001f,  -0.070f,  0.015f, 0},  
                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                
                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                {-0.0580f,  0.030f,  0.013f, 0},  
                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},

                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  

                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                {-0.0580f,  0.030f,  0.013f, 0},  
                                                {-0.0580f,  0.030f,  0.036f, 0},  

                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                {-0.001f,  -0.070f,  0.015f, 0},  
                                                {-0.001f,  -0.070f,  0.038f, 0},  

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                // For waiting
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   // count 51 
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  



                                                // 1 -> 9  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.038f, 1},  
                                                { 0.015f,  -0.012f,  0.038f, 1},  
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.038f, 0}, 

                                                // 3 -> 8  
                                                {-0.0583f, -0.035f,  0.035f, 0},  
                                                {-0.0583f, -0.035f,  0.012f, 1},  
                                                {-0.0583f, -0.035f,  0.012f, 1},  
                                                {-0.0583f, -0.035f,  0.012f, 1},  
                                                {-0.0583f, -0.035f,  0.035f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  

                                                // 5 -> 7  
 
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},   
                                                { 0.001f,   0.016f,  0.037f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.037f, 0},  

                                                 // 6 -> 10  
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.039f, 1},  
                                                {-0.0000f,  0.000f,  0.051f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.051f, 0},  // count = 94 

                                                // For waiting
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  // count = 97
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},    

                                                 // 10 -> 6  
                                                {-0.0000f,  0.000f,  0.049f, 0},  
                                                {-0.0000f,  0.000f,  0.031f, 1},  
                                                {-0.0000f,  0.000f,  0.031f, 1},  
                                                {-0.0000f,  0.000f,  0.031f, 1},  
                                                {-0.0000f,  0.000f,  0.049f, 1},  
                                                { 0.057f,   0.032f,  0.050f, 1},  
                                                { 0.057f,   0.032f,  0.017f, 1},  
                                                { 0.057f,   0.032f,  0.017f, 1},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},  

                                                // 7 -> 5  
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.037f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},   

                                                // 8 -> 3  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 1},  
                                                {-0.0583f, -0.035f,  0.037f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                // 9 -> 1  
                                                { 0.015f,  -0.012f,  0.038f, 0},  
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.038f, 1}, 
                                                { 0.055f,  -0.035f,  0.038f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},

                                                // For waiting
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0}
                                                };






#define MAX_MOTION_NUM 2
const float tool_position[MAX_MOTION_NUM][4] = {// { x, y, z}
                                                { 0.055f,  -0.0f,  0.088f, 0},  
                                                { 0.055f,  -0.0f,  0.015f, 0}
                                                };




uint8_t motion_cnt = 0;
uint8_t motion_page = 23;
// bool drawing_flag = true;
int drawing_flag = 0;
float start_angular_position = -PI/6.0f;
// const float move_time = 10.0;
float move_time = 5.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.065f;
float motion_no = 1;

bool motion[1] = {false};

void test()
{
  if (chain.moving() || chain.drawing())
  {
    return;
  }
  else
  {
    if (motion_no == 1){
    // if (~drawing_flag){  // why not working....???
      if (drawing_flag == 0){  // why not working....???
        if (motion_cnt == MAX_MOTION_NUM){
          motion_cnt = 0;
        }

        Pose target_pose;
        target_pose.position = OM_MATH::makeVector3(tool_position[motion_cnt][0]-0.04, 
                                                    tool_position[motion_cnt][1], 
                                                    tool_position[motion_cnt][2]+0.05);
        target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                      0.0f, 1.0f, 0.0f,
                                                      0.0f, 0.0f, 1.0f);

        chain.setPose(TOOL, target_pose, 0.5f);
        motion_cnt++;

      }
    }
  }
}

#endif // DEMO_H_
