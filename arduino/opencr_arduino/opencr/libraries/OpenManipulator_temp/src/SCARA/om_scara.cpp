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

#include "../../include/SCARA/om_scara.h"

OM_SCARA::OM_SCARA()
{}
OM_SCARA::~OM_SCARA()
{}


void OM_SCARA::initManipulator(bool using_platform)
{
  platform_ = using_platform;
  ////////// manipulator parameter initialization
  addWorld(WORLD,  // world name
           COMP1); // child name

  addComponent(COMP1, // my name
               WORLD, // parent name
               COMP2, // child name
               RM_MATH::makeVector3(-0.241, 0, 0.057), // relative position
               RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
               RM_MATH::makeVector3(0,0,1), // axis of rotation
               1); //actuator id
  addComponent(COMP2, // my name
               COMP1, // parent name
               COMP3, // child name           
               RM_MATH::makeVector3(0.067, 0.0, 0.0), // relative position
               RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
               RM_MATH::makeVector3(0,0,1), // axis of rotation
               2); // actuator id
  addComponent(COMP3, // my name
               COMP2, // parent name
               TOOL,  // child name 
               RM_MATH::makeVector3(0.067, 0.0, 0.0), // relative position
               RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
               RM_MATH::makeVector3(0,0,1), // axis of rotation
               3); // actuator id
  addTool(TOOL,  // my name
          COMP3, // parent name
          RM_MATH::makeVector3(0.107, 0.0, 0.0), // relative position
          RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
          15, // actuator id
          1.0);  // Change unit from `meter` to `radian`  <---??

  ////////// kinematics init.
  kinematics_ = new OM_KINEMATICS::SCARA();
  addKinematics(kinematics_);

  Serial.println("here??0.1");
  ////////// drawing path init
  addDrawingTrajectory(DRAWING_LINE, &line_);
  addDrawingTrajectory(DRAWING_CIRCLE, &circle_);
  addDrawingTrajectory(DRAWING_RHOMBUS, &rhombus_);
  addDrawingTrajectory(DRAWING_HEART, &heart_);
  Serial.println("here??0.111");

  if(platform_)
  {
    ////////// joint actuator init.
    actuator_ = new OM_DYNAMIXEL::JointDynamixel();
    // communication setting argument
    String dxl_comm_arg = "1000000";
    void *p_dxl_comm_arg = &dxl_comm_arg;
    
    jointDxlId.push_back(1);
    jointDxlId.push_back(2);
    jointDxlId.push_back(3);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

  Serial.println("here??0.2");
    // set joint actuator control mode
    String joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

  Serial.println("here??0.3");
    //----- tool actuator init -----//
    tool_ = new OM_DYNAMIXEL::GripperDynamixel();

    uint8_t gripperDxlId = 15;
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // set gripper actuator control mode
    String gripper_dxl_mode_arg = "position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

  Serial.println("here??0.4");
    //----- all actuator enable -----//
    allActuatorEnable();

    //----- manipulator trajectory & control time initialization -----//
    receiveAllJointActuatorValue();

    initTrajectoryWayPoint();
    setControlTime(CONTROL_TIME);
  }
}

void OM_SCARA::SCARAProcess(double present_time)
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

bool OM_SCARA::getPlatformFlag()
{
  return platform_;
}

