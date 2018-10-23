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

#ifndef OPEN_MANIPULATOR_DELTA_H_
#define OPEN_MANIPULATOR_DELTA_H_

// Necessary Library
#include <OpenManipulator.h>

// User-defined Library
#include <OMKinematics.h>
#include <OMDynamixel.h>

// Control Time Set 
#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.010f

// Define Components
#define WORLD 0
#define COMP1 1
#define COMP2 2  // motor id: 1
#define COMP3 3  // motor id: 2
#define COMP4 4  // motor id: 3
#define COMP5 5
#define COMP6 6
#define COMP7 7
#define COMP8 8
#define COMP9 9
#define COMP10 10
#define TOOL 11

#define NONE -1

#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

//-- Dynamixel --/
#define BAUD_RATE 1000000
#define DXL_SIZE 3

#define CIRCLE 11
#define RHOMBUS 13
#define HEART 15
#define CIRCLE2 12
#define RHOMBUS2 14
#define HEART2 16
#define SPIRAL 20
#define SPIRAL2 21

#define PLATFORM

OPEN_MANIPULATOR::OpenManipulator delta;

//-- Kinematics Init --//
OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Delta();


//////////////Suction Pin Num///////////////
// #define RELAY_PIN 8
// ////////////////////////////////////////////
// bool suction = true;

//-- Actuator Init --//
#ifdef PLATFORM 
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif 

OPEN_MANIPULATOR::Draw *heart = new OM_PATH::Heart();
OPEN_MANIPULATOR::Draw *spiral = new OM_PATH::Spiral();
OPEN_MANIPULATOR::Draw *spiral2 = new OM_PATH::Spiral2();

void initManipulator()
{
  delta.addWorld(WORLD,
                 COMP1);

  delta.addComponent(COMP1,
                     WORLD,
                     COMP2,
                     OM_MATH::makeVector3(0.000f, 0.000f, 0.169894f),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     -1);

  delta.addComponent(COMP2,
                     COMP1,
                     COMP5,
                     OM_MATH::makeVector3(-0.055f, 0.000f, 0.0),
                     Eigen::Matrix3f::Identity(3, 3));

  delta.addComponent(COMP3,
                     COMP1,
                     COMP6,
                     OM_MATH::makeVector3(0.0275f, -0.0476f, 0.0), // should be modified
                     Eigen::Matrix3f::Identity(3, 3));

  delta.addComponent(COMP4,
                     COMP1,
                     COMP7,
                     OM_MATH::makeVector3(0.0275f, 0.0476f, 0.0), // should be modified
                     Eigen::Matrix3f::Identity(3, 3));

  delta.addComponent(COMP5,
                     COMP2,
                     COMP8,
                     OM_MATH::makeVector3(-0.100f, 0.000f, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     1);

  delta.addComponent(COMP6,
                     COMP3,
                     COMP9,
                     OM_MATH::makeVector3(0.050f, -0.0866f, 0.0), // should be modified
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     2);

  delta.addComponent(COMP7,
                     COMP4,
                     COMP10,
                     OM_MATH::makeVector3(0.050f, 0.0866f, 0.0), // should be modified
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     3);

  delta.addComponent(COMP8,
                     COMP5,
                     TOOL,
                     OM_MATH::makeVector3(0.135f, 0.000f, -0.169894f), 
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     -1);

  delta.addTool(TOOL,    // why defined like this???
                COMP8,
                OM_MATH::makeVector3(0.020f, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3));

  delta.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  Serial.println("hahaha1??");
  Serial.flush();;
  delta.initActuator(actuator);
  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  Serial.println("hahaha2??");
  Serial.flush();;
  delta.actuatorInit(p_baud_rate);
  Serial.println("hahaha3??");
  Serial.flush();;
  delta.actuatorEnable();
  Serial.println("hahaha31??");
  Serial.flush();;
#endif /////////////////////////////////////////////
  delta.initJointTrajectory();
  delta.setControlTime(ACTUATOR_CONTROL_TIME);
  Serial.println("hahaha4??");
  Serial.flush();;

#ifdef PLATFORM ////////////////////////////////////Actuator init    
  std::vector<float> goal_position_;   // rename this var name
  goal_position_.push_back(0.0f);
  goal_position_.push_back(0.0f);
  goal_position_.push_back(0.0f);
  delta.jointMove(goal_position_, 1.0f);
  Serial.println("hahaha5??");
  Serial.flush();;

  delta.setAllActiveJointAngle(delta.receiveAllActuatorAngle());
#endif /////////////////////////////////////////////
  // delta.forward(COMP1);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  delta.setAllActiveJointAngle(delta.receiveAllActuatorAngle());
#endif
}

// void THREAD::Robot_State(void const *argument)
// {
//   (void)argument;

//   for (;;)
//   {
//     MUTEX::wait();

//     updateAllJointAngle();
//     // delta.forward(COMP1);

//     MUTEX::release();

//     osDelay(ROBOT_STATE_UPDATE_TIME * 1000);
//   }
// }

// void THREAD::Actuator_Control(void const *argument)
// {
//   (void)argument;

//   for (;;)
//   {
//     MUTEX::wait();

//     delta.jointControl();

//     MUTEX::release();

//     osDelay(ACTUATOR_CONTROL_TIME * 1000);
//   }
// }
#endif //OPEN_MANIPULATOR_DELTA_H_
