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

#include "Planar.h"

#define MAX_MOTION_NUM 5

uint8_t motion_erase = 1;
uint8_t motion_page = 15;
uint8_t motion_repeat = 0;

float start_angular_position = 0.0f;
// const float move_time = 10.0f;
float move_time = 10.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.020f;

bool motion[1] = {false};

void test2()
{
  if (planar.moving() || planar.drawing())
  {
    return;
  }
  else
  {
    if (motion_page == SPIRAL)
    {
      radius = 0.013f;
      // planar.drawInit(HEART, move_time, p_init_arg);
      // planar.setRadiusForDrawing(HEART, radius);
      // planar.setStartPositionForDrawing(HEART, planar.getComponentPositionToWorld(TOOL));
      // planar.setStartAngularPositionForDrawing(HEART, start_angular_position);
      // planar.draw(HEART);
      planar.drawInit(SPIRAL, move_time, p_init_arg);
      planar.setRadiusForDrawing(SPIRAL, radius);
      planar.setStartPositionForDrawing(SPIRAL, planar.getComponentPositionToWorld(TOOL));
      planar.setStartAngularPositionForDrawing(SPIRAL, start_angular_position);
      planar.draw(SPIRAL);

      motion_page++;
    }
    else
    {
      radius = 0.013f;
      planar.drawInit(SPIRAL2, move_time, p_init_arg);
      planar.setRadiusForDrawing(SPIRAL2, radius);
      planar.setStartPositionForDrawing(SPIRAL2, planar.getComponentPositionToWorld(TOOL));
      planar.setStartAngularPositionForDrawing(SPIRAL2, start_angular_position);
      planar.draw(SPIRAL2);
      motion_page=20;
    }
  }
}




void test3()
{
  if (planar.moving() || planar.drawing())
  {
    return;
  }
  else
  {
    if (motion_page == HEART)
    {
      radius = 0.040f;
      move_time = 20.0f;
      init_arg[0] = move_time;
      p_init_arg = &init_arg;
      // planar.drawInit(HEART, move_time, p_init_arg);
      // planar.setRadiusForDrawing(HEART, radius);
      // planar.setStartPositionForDrawing(HEART, planar.getComponentPositionToWorld(TOOL));
      // planar.setStartAngularPositionForDrawing(HEART, start_angular_position);
      // planar.draw(HEART);
      planar.drawInit(HEART, move_time, p_init_arg);
      planar.setRadiusForDrawing(HEART, radius);
      planar.setStartPositionForDrawing(HEART, planar.getComponentPositionToWorld(TOOL));
      planar.setStartAngularPositionForDrawing(HEART, start_angular_position);
      planar.draw(HEART);

      // motion_page++;
    }
    // else
    // {
    //   radius = 0.013f;
    //   planar.drawInit(SPIRAL2, move_time, p_init_arg);
    //   planar.setRadiusForDrawing(SPIRAL2, radius);
    //   planar.setStartPositionForDrawing(SPIRAL2, planar.getComponentPositionToWorld(TOOL));
    //   planar.setStartAngularPositionForDrawing(SPIRAL2, start_angular_position);
    //   planar.draw(SPIRAL2);
    //   motion_page=20;
    // }
  }
}

// void test3()
// {
//   if (planar.moving() || planar.drawing())
//   {
//     return;
//   }
//   else
//   {
//     if (motion_page == SPIRAL)
//     {
//       radius = 0.013f;
//       // planar.drawInit(HEART, move_time, p_init_arg);
//       // planar.setRadiusForDrawing(HEART, radius);
//       // planar.setStartPositionForDrawing(HEART, planar.getComponentPositionToWorld(TOOL));
//       // planar.setStartAngularPositionForDrawing(HEART, start_angular_position);
//       // planar.draw(HEART);
//       planar.drawInit(SPIRAL, move_time, p_init_arg);
//       planar.setRadiusForDrawing(SPIRAL, radius);
//       planar.setStartPositionForDrawing(SPIRAL, planar.getComponentPositionToWorld(TOOL));
//       planar.setStartAngularPositionForDrawing(SPIRAL, start_angular_position);
//       planar.draw(SPIRAL);

//       motion_page++;
//     }
//     else
//     {
//       radius = 0.013f;
//       planar.drawInit(SPIRAL2, move_time, p_init_arg);
//       planar.setRadiusForDrawing(SPIRAL2, radius);
//       planar.setStartPositionForDrawing(SPIRAL2, planar.getComponentPositionToWorld(TOOL));
//       planar.setStartAngularPositionForDrawing(SPIRAL2, start_angular_position);
//       planar.draw(SPIRAL2);
//       motion_page=20;
//     }
//   }
// }




void test2Stop()
{

  Vector3f pos;
  pos(0) = 0.0;
  pos(1) = 0.0;
  pos(2) = 0.0;
  planar.drawLine(TOOL, pos, 1.5); 
  // std::vector<float> target_angle_vector;
  // planar.setMove(TOOL, OM_MATH::makeVector3(0.000f, 0.000f, 0.0), 1.0);

  // target_angle_vector.push_back(0.0);
  // target_angle_vector.push_back(0.0);
  // target_angle_vector.push_back(0.0);
  // target_angle_vector.push_back(PI*7.0f/12.0f);
  // target_angle_vector.push_back(PI*7.0f/12.0f);
  // target_angle_vector.push_back(PI*7.0f/12.0f);
  // target_angle_vector.push_back(-PI*3.0f);
  // planar.jointMove(target_angle_vector, 1.0);
  motion_page = 20;
}

#endif // DEMO_H_
