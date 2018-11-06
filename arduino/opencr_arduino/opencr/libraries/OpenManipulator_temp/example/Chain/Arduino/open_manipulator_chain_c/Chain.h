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

#ifndef OPEN_MANIPULATOR_CHAIN_H_
#define OPEN_MANIPULATOR_CHAIN_H_

// Necessary library
#include <OpenManipulator.h>

// User-defined library
#include <OMKinematics.h>
#include <OMDynamixel.h>
#include <OMDebug.h>

#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.010f

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define TOOL 5

#define NONE -1


#define CIRCLE 11
#define RHOMBUS 13
#define HEART 15
#define CIRCLE2 12
#define RHOMBUS2 14
#define HEART2 16
#define SPIRAL 20
#define BOTTLESHAKE 17
#define BOTTLESHAKEY 18
#define BOTTLESHAKEX 19
#define BOTTLESHAKE3 20
#define SPIRAL2 21
#define CIRCLEEDGE 22
#define CIRCLEEDGE2 23


#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

#define BAUD_RATE 1000000
//#define DXL_SIZE 5

#define ACTIVE_JOINT_SIZE 4

#define PLATFORM

OPEN_MANIPULATOR::OpenManipulator chain;

OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Chain();
#ifdef PLATFORM ////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif /////////////////////////////////////////////
//  OPEN_MANIPULATOR::Path *path = new MY_PATH::Circle();


OPEN_MANIPULATOR::Draw *circle = new OM_PATH::Circle();
OPEN_MANIPULATOR::Draw *rhombus = new OM_PATH::Rhombus();
OPEN_MANIPULATOR::Draw *heart = new OM_PATH::Heart();
OPEN_MANIPULATOR::Draw *bottleshake = new OM_PATH::BottleShake();
OPEN_MANIPULATOR::Draw *bottleshakeY = new OM_PATH::BottleShakeY();
OPEN_MANIPULATOR::Draw *bottleshakeX = new OM_PATH::BottleShakeX();
OPEN_MANIPULATOR::Draw *bottleshake3 = new OM_PATH::BottleShake3();

OPEN_MANIPULATOR::Draw *spiral = new OM_PATH::Spiral();
OPEN_MANIPULATOR::Draw *spiral2 = new OM_PATH::Spiral2();
OPEN_MANIPULATOR::Draw *circleedge = new OM_PATH::CircleEdge();
OPEN_MANIPULATOR::Draw *circleedge2 = new OM_PATH::CircleEdge2();



void initManipulator()
{
  chain.addWorld(WORLD,
                 COMP1);

  chain.addComponent(COMP1,
                     WORLD,
                     COMP2,
                     OM_MATH::makeVector3(-0.278, 0.0, 0.017),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     11);

  chain.addComponent(COMP2,
                     COMP1,
                     COMP3,
                     OM_MATH::makeVector3(0.0, 0.0, 0.058),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     12);

  chain.addComponent(COMP3,
                     COMP2,
                     COMP4,
                     OM_MATH::makeVector3(0.024, 0.0, 0.128),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     13);

  chain.addComponent(COMP4,
                     COMP3,
                     TOOL,
                     OM_MATH::makeVector3(0.124, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     14);

  chain.addTool(TOOL,
                COMP4,
                OM_MATH::makeVector3(0.130, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3),
                15,
                1.0f); // Change unit from `meter` to `radian`

  chain.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  chain.initActuator(actuator);

  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  chain.actuatorInit(p_baud_rate);
  chain.setActuatorControlMode();
  chain.actuatorEnable();
#endif /////////////////////////////////////////////
  chain.initJointTrajectory();
  chain.setControlTime(ACTUATOR_CONTROL_TIME);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  chain.toolMove(TOOL, 0.0f);
  chain.setAllActiveJointAngle(chain.receiveAllActuatorAngle());
#endif /////////////////////////////////////////////
  chain.forward(COMP1);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  chain.setAllActiveJointAngle(chain.receiveAllActuatorAngle());
#endif
  // Add passive joint function
}

#endif //OPEN_MANIPULATOR_CHAIN_H_
