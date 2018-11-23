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

#ifndef DEMO2_H_
#define DEMO2_H_

#include <Delta.h>

bool suction_on = false;

#define MAX_MOTION_NUM 118

float hall[10][2] = { 
                  { 0.0580f, -0.0340f}, // Hall 1 
                  { 0.0000f, -0.0690f}, // Hall 2
                  {-0.0563f, -0.0360f}, // Hall 3
                  {-0.0570f,  0.0315f}, // Hall 4
                  {-0.0000f,  0.0660f}, // Hall 5
                  { 0.0580f,  0.0335f}, // Hall 6
                  { 0.0000f,  0.0173f}, // Hall 7
                  {-0.0150f, -0.0087f}, // Hall 8
                  { 0.0150f, -0.0087f}, // Hall 9
                  { 0.0000f,  0.0000f}  // Centre
                    };

const float tool_position[MAX_MOTION_NUM][4] = {// { x, y, z}

                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  
                                                {hall[9][0], hall[9][1], 0.040f, 0},  

                                                // Hall 1
                                                {hall[0][0], hall[0][1], 0.040f, 0},  
                                                {hall[0][0], hall[0][1], 0.023f, 0},  
                                                {hall[0][0], hall[0][1], 0.040f, 0},  
                                                // Hall 2
                                                {hall[1][0], hall[1][1], 0.040f, 0},  
                                                {hall[1][0], hall[1][1], 0.023f, 0},  
                                                {hall[1][0], hall[1][1], 0.040f, 0},  
                                                // Hall 3                                                
                                                {hall[2][0], hall[2][1], 0.040f, 0},  
                                                {hall[2][0], hall[2][1], 0.023f, 0},  
                                                {hall[2][0], hall[2][1], 0.040f, 0},  
                                                // Hall 4
                                                {hall[3][0], hall[3][1], 0.040f, 0},  
                                                {hall[3][0], hall[3][1], 0.023f, 0},  
                                                {hall[3][0], hall[3][1], 0.040f, 0},                                                 
                                                // Hall 5
                                                {hall[4][0], hall[4][1], 0.040f, 0},  
                                                {hall[4][0], hall[4][1], 0.023f, 0},  
                                                {hall[4][0], hall[4][1], 0.040f, 0},  
                                                // Hall 6
                                                {hall[5][0], hall[5][1], 0.040f, 0},  
                                                {hall[5][0], hall[5][1], 0.023f, 0},  
                                                {hall[5][0], hall[5][1], 0.040f, 0},
                                                // Hall 5
                                                {hall[4][0], hall[4][1], 0.040f, 0},  
                                                {hall[4][0], hall[4][1], 0.023f, 0},  
                                                {hall[4][0], hall[4][1], 0.040f, 0},  
                                                // Hall 4
                                                {hall[3][0], hall[3][1], 0.040f, 0},  
                                                {hall[3][0], hall[3][1], 0.023f, 0},  
                                                {hall[3][0], hall[3][1], 0.040f, 0},                                                 
                                                // Hall 3                                                
                                                {hall[2][0], hall[2][1], 0.040f, 0},  
                                                {hall[2][0], hall[2][1], 0.023f, 0},  
                                                {hall[2][0], hall[2][1], 0.040f, 0},  
                                                // Hall 2
                                                {hall[1][0], hall[1][1], 0.040f, 0},  
                                                {hall[1][0], hall[1][1], 0.023f, 0},  
                                                {hall[1][0], hall[1][1], 0.040f, 0},  
                                                // Hall 1
                                                {hall[0][0], hall[0][1], 0.040f, 0},  
                                                {hall[0][0], hall[0][1], 0.023f, 0},  
                                                {hall[0][0], hall[0][1], 0.040f, 0},  

                                                // 1 -> 9  
                                                {hall[0][0], hall[0][1], 0.040f, 0},  
                                                {hall[0][0], hall[0][1], 0.023f, 1},  
                                                {hall[0][0], hall[0][1], 0.023f, 1},  
                                                {hall[0][0], hall[0][1], 0.023f, 1},  
                                                {hall[0][0], hall[0][1], 0.040f, 1},  
                                                {hall[8][0], hall[8][1], 0.040f, 1},  
                                                {hall[8][0], hall[8][1], 0.023f, 1}, 
                                                {hall[8][0], hall[8][1], 0.023f, 1}, 
                                                {hall[8][0], hall[8][1], 0.023f, 0}, 
                                                {hall[8][0], hall[8][1], 0.040f, 0}, 
                                                // 3 -> 8  
                                                {hall[2][0], hall[2][1], 0.040f, 0},  
                                                {hall[2][0], hall[2][1], 0.023f, 1},  
                                                {hall[2][0], hall[2][1], 0.023f, 1},  
                                                {hall[2][0], hall[2][1], 0.023f, 1},  
                                                {hall[2][0], hall[2][1], 0.040f, 1},  
                                                {hall[7][0], hall[7][1], 0.040f, 1},  
                                                {hall[7][0], hall[7][1], 0.023f, 1},  
                                                {hall[7][0], hall[7][1], 0.023f, 1},  
                                                {hall[7][0], hall[7][1], 0.023f, 0},  
                                                {hall[7][0], hall[7][1], 0.023f, 0},  
                                                // 5 -> 7  
                                                {hall[4][0], hall[4][1], 0.040f, 0},  
                                                {hall[4][0], hall[4][1], 0.023f, 1},  
                                                {hall[4][0], hall[4][1], 0.023f, 1},  
                                                {hall[4][0], hall[4][1], 0.023f, 1},  
                                                {hall[4][0], hall[4][1], 0.040f, 1},   
                                                {hall[6][0], hall[6][1], 0.040f, 1},  
                                                {hall[6][0], hall[6][1], 0.023f, 1},  
                                                {hall[6][0], hall[6][1], 0.023f, 1},  
                                                {hall[6][0], hall[6][1], 0.023f, 0},  
                                                {hall[6][0], hall[6][1], 0.040f, 0},  
                                                 // 6 -> 10  
                                                {hall[5][0], hall[5][1], 0.040f, 0},  
                                                {hall[5][0], hall[5][1], 0.023f, 1},  
                                                {hall[5][0], hall[5][1], 0.023f, 1},  
                                                {hall[5][0], hall[5][1], 0.023f, 1},  
                                                {hall[5][0], hall[5][1], 0.055f, 1},  
                                                {hall[9][0], hall[9][1], 0.055f, 1},  
                                                {hall[9][0], hall[9][1], 0.042f, 1},  
                                                {hall[9][0], hall[9][1], 0.042f, 1},  
                                                {hall[9][0], hall[9][1], 0.042f, 0},  
                                                {hall[9][0], hall[9][1], 0.055f, 0},  // count = 94 

                                                // For waiting
                                                {hall[5][0], hall[5][1], 0.025f, 0},  
                                                {hall[5][0], hall[5][1], 0.025f, 0},  
                                                {hall[5][0], hall[5][1], 0.025f, 0},  
                                                {hall[5][0], hall[5][1], 0.040f, 0},  
                                                {hall[5][0], hall[5][1], 0.025f, 0},  
                                                {hall[5][0], hall[5][1], 0.040f, 0},  
                                                {hall[5][0], hall[5][1], 0.010f, 0}  
                                                };

uint8_t motion_cnt = 0;
uint8_t motion_page = 23;
// bool drawing_flag = true;
int drawing_flag = 0;
float start_angular_position = -PI/6.0f;
// const float move_time = 10.0;
float move_time = 2.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.065f;
float motion_no = 1;
bool motion[1] = {false};

void test2()
{
  if (delta.moving() || delta.drawing())
  {
    return;
  }
  else
  {
    // if (~drawing_flag){  // why not working....???
      if (drawing_flag == 0){  // why not working....???
        if (motion_cnt == MAX_MOTION_NUM){
          motion_cnt = 0;
        }

        Pose target_pose;
        target_pose.position = OM_MATH::makeVector3(tool_position[motion_cnt][0], 
                                                    tool_position[motion_cnt][1], 
                                                    tool_position[motion_cnt][2]);
        target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                      0.0f, 1.0f, 0.0f,
                                                      0.0f, 0.0f, 1.0f);
        if (motion_cnt <= 43) delta.setPose(TOOL, target_pose, 0.07f);
        else delta.setPose(TOOL, target_pose, 0.15f);

        if (tool_position[motion_cnt][3] == 1){
          pinMode(4, OUTPUT);
          pinMode(7, OUTPUT);
          pinMode(RELAY_PIN, OUTPUT);
          pinMode(12, OUTPUT);
          digitalWrite(RELAY_PIN, HIGH);      //suction on
          digitalWrite(12, HIGH);      //suction on
          suction_on = true;
        } 
        else{
          pinMode(4, OUTPUT);
          pinMode(7, OUTPUT);
          pinMode(RELAY_PIN, OUTPUT);
          pinMode(12, OUTPUT);
          digitalWrite(RELAY_PIN, LOW);      //suction off
          digitalWrite(12, LOW);      //suction on
          suction_on = false;
        }

        motion_cnt++;

        if (motion_cnt == 85){
          // drawing = true;  // why not wokring...?
          drawing_flag = 1;
        }
      }

      else 
      {
        Vector3f temp_current_position;
        temp_current_position(0) = hall[0][0];
        temp_current_position(1) = hall[0][1];
        temp_current_position(2) = 0.040f;

        if (motion_page == CIRCLEEDGE2)
        {
          delta.drawInit(CIRCLEEDGE2, move_time, p_init_arg);
          delta.setRadiusForDrawing(CIRCLEEDGE2, radius);
          delta.setStartPositionForDrawing(CIRCLEEDGE2, temp_current_position);
          delta.setStartAngularPositionForDrawing(CIRCLEEDGE2, start_angular_position);
          delta.draw(CIRCLEEDGE2);
          drawing_flag = 0;
          motion_page = 22;
        }
        else
        {
          delta.drawInit(CIRCLEEDGE, move_time, p_init_arg);
          delta.setRadiusForDrawing(CIRCLEEDGE, radius);
          delta.setStartPositionForDrawing(CIRCLEEDGE, temp_current_position);
          delta.setStartAngularPositionForDrawing(CIRCLEEDGE, start_angular_position);
          delta.draw(CIRCLEEDGE);
          motion_page++;
          // drawing = false;  // why not working??
          drawing_flag = 0;
        }
      }
  } 
}

#endif // DEMO2_H_
