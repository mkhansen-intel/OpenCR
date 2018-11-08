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

#ifndef DEMO_H_
#define DEMO_H_

#include "om_scara_lib.h"

#define MAX_MOTION_NUM 5

uint8_t motion_erase = 1;
uint8_t motion_page = 1;
uint8_t motion_repeat = 0;

double start_angular_position = 0.0f;
const double move_time = 3.0f;
double init_arg[2] = {move_time, CONTROL_TIME};
void *p_init_arg = init_arg;
double radius = 0.020f;

void test(OM_SCARA *SCARA)
{
  // if (SCARA->isMoving() || SCARA->drawing()) {
  if (SCARA->isMoving()) {
    Serial.println("here??1.1");
    Serial.flush();
    return;
  }
  else {

    // Erasing
    Serial.println("here??1.2");
    Serial.flush();
    if (motion_erase == 1){
      if (motion_repeat == 0){
        Serial.println("here??1.2.1");
        Serial.flush();
        SCARA->toolMove(TOOL, -0.5f);
        motion_repeat++;
        Serial.println("here??1.3");
        Serial.flush();
      }    
      else if (motion_repeat == 1){
        std::vector<double> goal_position;
        goal_position.push_back(-2.17);
        goal_position.push_back(0.82);
        goal_position.push_back(2.05);

        SCARA->jointTrajectoryMove(goal_position, 3.0f); 
        motion_repeat++;
      }    
      else if (motion_repeat == 2){
        SCARA->toolMove(TOOL, -0.0f);

        std::vector<double> goal_position;
        goal_position.push_back(-2.17);
        goal_position.push_back(0.82);
        goal_position.push_back(2.05);
        SCARA->jointTrajectoryMove(goal_position, 3.0f); 

        motion_repeat++;
      }    
      else if (motion_repeat == 3){
        SCARA->toolMove(TOOL, -0.5f);

        std::vector<double> goal_position;
        goal_position.push_back(-2.17);
        goal_position.push_back(0.82);
        goal_position.push_back(2.05);
        SCARA->jointTrajectoryMove(goal_position, 3.0f); 

        motion_repeat++;
      }    
      else {
        std::vector<double> goal_position;

        if (motion_page == DRAWING_CIRCLE){
          goal_position.push_back(-1.05);
          goal_position.push_back(0.9);
          goal_position.push_back(0.9);
          SCARA->jointTrajectoryMove(goal_position, 2.0f); 
        }
        else if (motion_page == DRAWING_CIRCLE2) {
          goal_position.push_back(-1.45);
          goal_position.push_back(1.2);
          goal_position.push_back(1.2);
          SCARA->jointTrajectoryMove(goal_position, 2.0f); 
        }
        else if (motion_page == DRAWING_RHOMBUS || motion_page == DRAWING_RHOMBUS2) {
          goal_position.push_back(-1.80);
          goal_position.push_back(1.43);
          goal_position.push_back(1.43);
          SCARA->jointTrajectoryMove(goal_position, 2.0f); 
        }

        else {
          goal_position.push_back(-1.6);
          goal_position.push_back(1.3);
          goal_position.push_back(1.3);
          SCARA->jointTrajectoryMove(goal_position, 2.0f); 
        }
        motion_erase = 0;
        motion_repeat = 0;
      }
    }

    // Drawing
    else {
      SCARA->toolMove(TOOL, -0.0f);

      if (motion_page == DRAWING_CIRCLE) {
        // SCARA->drawInit(CIRCLE, move_time, p_init_arg);
        // SCARA->setRadiusForDrawing(CIRCLE, radius);  
        // SCARA->setStartPositionForDrawing(CIRCLE, SCARA->getComponentPositionToWorld(TOOL));
        // SCARA->setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
        // SCARA->drawingTrajectoryMove(CIRCLE);
        SCARA->taskTrajectoryMoveToPresentPosition(TOOL, RM_MATH::makeVector3(-0.030, 0.0, 0.0), 2.0);
        // double draw_circle_arg[3];
        // draw_circle_arg[0] = 0.035; // Radius (m)
        // draw_circle_arg[1] = 1;     // Number of revolution
        // draw_circle_arg[2] = 0.0;   // Starting angular position (rad)
        // void* p_draw_circle_arg = &draw_circle_arg;
        // SCARA->drawingTrajectoryMove(DRAWING_CIRCLE, TOOL, p_draw_circle_arg, 10.0);

        motion_erase = 1;
        motion_page = DRAWING_CIRCLE2;
      }
      else if (motion_page == DRAWING_CIRCLE2) { 
        // SCARA->drawInit(CIRCLE, move_time, p_init_arg);
        // SCARA->setRadiusForDrawing(CIRCLE, radius);  
        // SCARA->setStartPositionForDrawing(CIRCLE, SCARA->getComponentPositionToWorld(TOOL));
        // SCARA->setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
        double draw_circle_arg[3];
        draw_circle_arg[0] = 0.020; // Radius (m)
        draw_circle_arg[1] = 1;     // Number of revolution
        draw_circle_arg[2] = start_angular_position; // Starting angular position (rad)
        void* p_draw_circle_arg = &draw_circle_arg;
        SCARA->drawingTrajectoryMove(DRAWING_CIRCLE, TOOL, p_draw_circle_arg, 10.0);

        motion_repeat++;
        start_angular_position = start_angular_position + 2*PI/6;

        if (motion_repeat == 6){
          motion_erase = 1;
          motion_page = DRAWING_RHOMBUS;
          motion_repeat = 0;
          start_angular_position = 0;
        }
      } 
      else if (motion_page == DRAWING_RHOMBUS) {
        // SCARA->drawInit(RHOMBUS, move_time, p_init_arg);
        // SCARA->setRadiusForDrawing(RHOMBUS, radius);  
        // SCARA->setStartPositionForDrawing(RHOMBUS, SCARA->getComponentPositionToWorld(TOOL));
        // SCARA->setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
        double draw_circle_arg[3];
        draw_circle_arg[0] = 0.035; // Radius (m)
        draw_circle_arg[1] = 1;     // Number of revolution
        draw_circle_arg[2] = PI; // Starting angular position (rad)
        void* p_draw_circle_arg = &draw_circle_arg;
        SCARA->drawingTrajectoryMove(DRAWING_RHOMBUS, TOOL, p_draw_circle_arg, 10.0);

        motion_erase = 1;
        motion_page = DRAWING_RHOMBUS2;
      } 
      else if (motion_page == DRAWING_RHOMBUS2) {
        // SCARA->drawInit(RHOMBUS, move_time, p_init_arg);
        // SCARA->setRadiusForDrawing(RHOMBUS, radius);  
        // SCARA->setStartPositionForDrawing(RHOMBUS, SCARA->getComponentPositionToWorld(TOOL));
        // SCARA->setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
        double draw_circle_arg[3];
        draw_circle_arg[0] = radius; // Radius (m)
        draw_circle_arg[1] = 1;     // Number of revolution
        draw_circle_arg[2] = PI; // Starting angular position (rad)
        void* p_draw_circle_arg = &draw_circle_arg;
        SCARA->drawingTrajectoryMove(DRAWING_RHOMBUS, TOOL, p_draw_circle_arg, 10.0);

        radius += 0.007f;
        motion_repeat++;

        if (motion_repeat == 3){
          motion_erase = 1;
          motion_page = DRAWING_HEART;
          motion_repeat = 0;
          radius = 0.020f;
        }
      } 
      else if (motion_page == DRAWING_HEART) { 
        // SCARA->drawInit(HEART, move_time, p_init_arg);
        // SCARA->setRadiusForDrawing(HEART, radius);  
        // SCARA->setStartPositionForDrawing(HEART, SCARA->getComponentPositionToWorld(TOOL));
        // SCARA->setStartAngularPositionForDrawing(HEART, start_angular_position);
        double draw_circle_arg[3];
        draw_circle_arg[0] = 0.045; // Radius (m)
        draw_circle_arg[1] = 1;     // Number of revolution
        draw_circle_arg[2] = PI; // Starting angular position (rad)
        void* p_draw_circle_arg = &draw_circle_arg;
        SCARA->drawingTrajectoryMove(DRAWING_HEART, TOOL, p_draw_circle_arg, 10.0);

        motion_erase = 1;
        motion_page = DRAWING_CIRCLE;
      } 
      else
        motion_page = DRAWING_CIRCLE;
    }
  }
}

#endif // DEMO_H_
