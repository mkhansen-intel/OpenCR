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

#include "../../include/Link/om_link.h"

OM_LINK::OM_LINK()
{}
OM_LINK::~OM_LINK()
{}

void OM_LINK::initManipulator(bool using_platform, bool using_processing)
{
  platform_ = using_platform;
  processing_ = using_processing;

  ////////// manipulator parameter initialization
  addWorld(WORLD, JOINT0);
  addComponent(JOINT0, WORLD, JOINT1,
                           RM_MATH::makeVector3(-0.23867882, 0, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,0,1),
                           1,
                           1.0);
  addComponentChild(JOINT0, JOINT2);
  addComponentChild(JOINT0, JOINT7);
  addComponent(JOINT1, JOINT0, JOINT5, 
                           RM_MATH::makeVector3(0, 0.022, 0.052),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0),
                           2,
                           -1.0);
  addComponent(JOINT2, JOINT0, JOINT3,
                           RM_MATH::makeVector3(0, -0.022, 0.052),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0),
                           3,
                           1.0);
  addComponent(JOINT3, JOINT2, JOINT4,
                           RM_MATH::makeVector3(0.050, 0.007, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT4, JOINT3, JOINT5,
                           RM_MATH::makeVector3(0.200, 0.006, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT5, JOINT1, JOINT6,
                           RM_MATH::makeVector3(0.200, -0.016, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT6, JOINT5, SUCTION,
                           RM_MATH::makeVector3(0.200, -0.009, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT7, JOINT0, JOINT8,
                           RM_MATH::makeVector3(-0.04531539, 0.006, 0.07313091),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT8, JOINT7, JOINT9,
                           RM_MATH::makeVector3(0.200, 0.009, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT9, JOINT8, JOINT10,
                           RM_MATH::makeVector3(0.07660444, -0.006, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addComponent(JOINT10, JOINT9, SUCTION,
                           RM_MATH::makeVector3(0.200, -0.006, 0),
                           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                           RM_MATH::makeVector3(0,1,0));
  addTool(SUCTION, JOINT6,
                      RM_MATH::makeVector3(0.03867882, 0.003, -0.01337315-0.01),
                      RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0),
                      4,
                      1.0);

  ////////// kinematics init.
  kinematics_ = new OM_KINEMATICS::Link();
  addKinematics(kinematics_);

  ////////// joint actuator init.
  actuator_ = new OM_DYNAMIXEL::JointDynamixel();
  String dxl_comm_arg = "1000000";
  void *p_dxl_comm_arg = &dxl_comm_arg;
  addJointActuator(JOINT_DYNAMIXEL, actuator_, getManipulator()->getAllActiveJointID(), p_dxl_comm_arg);
  String joint_dxl_mode_arg = "position_mode";
  void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
  jointActuatorSetMode(JOINT_DYNAMIXEL, getManipulator()->getAllActiveJointID(), p_joint_dxl_mode_arg);

  ////////// tool actuator init.
  tool_ = new SuctionModule();
  uint8_t suc_pin_arg = RELAY_PIN;
  void *p_suc_pin_arg = &suc_pin_arg;
  addToolActuator(SUCTION_MODULE, tool_, getManipulator()->getToolId(SUCTION), p_suc_pin_arg);

  ////////// drawing path init
  addDrawingTrajectory(DRAWING_LINE, &line_);
  addDrawingTrajectory(DRAWING_CIRCLE, &circle_);
  addDrawingTrajectory(DRAWING_RHOMBUS, &rhombus_);
  addDrawingTrajectory(DRAWING_HEART, &heart_);

  // all actuator enable
  //allActuatorEnable();

  ////////// manipulator trajectory & control time initialization
  receiveAllJointActuatorValue();

  initTrajectoryWayPoint();
  setControlTime(ACTUATOR_CONTROL_TIME);
  //////////////////////////////////////////////////////////
}

void OM_LINK::Process(double present_time)
{
  std::vector<WayPoint> goal_value = trajectoryControllerLoop(present_time);

  if(platform_)
  {
    receiveAllJointActuatorValue();
    if(goal_value.size() != 0)  sendAllJointActuatorValue(goal_value);
    forward();
  }
  else
  {
    if(goal_value.size() != 0) setAllActiveJointValue(goal_value); // visualization
    forward();
  } 
}

bool OM_LINK::getPlatformFlag()
{
  return platform_;
}

bool OM_LINK::getProcessingFlag()
{
  return processing_;
}