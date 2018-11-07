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

#ifndef OM_LINK_H_
#define OM_LINK_H_

// Necessary library
// #include <robotis_manipulator.h>
#include <om_link_lib.h>
#include <RTOS.h>
// User-defined library

/////////////control time set////////////////
#define ACTUATOR_CONTROL_TIME 0.010
////////////////////////////////////////////

/////////////////////NAME/////////////////////
#define WORLD     0
#define JOINT0    1
#define JOINT1    2
#define JOINT2    3
#define JOINT3    4
#define JOINT4    5
#define JOINT5    6
#define JOINT6    7
#define JOINT7    8
#define JOINT8    9
#define JOINT9    10
#define JOINT10   11
#define SUCTION   12
////////////////////////////////////////////

//////////////////Actuator//////////////////
#define JOINT_DYNAMIXEL 0
#define SUCTION_MODULE 1
////////////////////////////////////////////

//////////////Suction Pin Num///////////////
#define RELAY_PIN 8
////////////////////////////////////////////

//////////////////Drawing///////////////////
#define DRAWING_LINE 0
#define DRAWING_CIRCLE 1
#define DRAWING_RHOMBUS 2
#define DRAWING_HEART 3
////////////////////////////////////////////

////////////////////////////////////////////
#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)
////////////////////////////////////////////

class OM_LINK : public ROBOTIS_MANIPULATOR::RobotisManipulator
{
 private:
  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::JointActuator *actuator_;
  ROBOTIS_MANIPULATOR::ToolActuator *tool_;

  OM_DRAWING::Line line_;
  OM_DRAWING::Circle circle_;
  OM_DRAWING::Rhombus rhombus_;
  OM_DRAWING::Heart heart_;

  bool platform_;
  bool processing_;

 public:
  OM_LINK();
  virtual ~OM_LINK();

  void initManipulator(bool using_platform, bool using_processing);
  void Process(double present_time);
  bool getPlatformFlag();
  bool getProcessingFlag();  
};

#endif //OM_LINK_H_
