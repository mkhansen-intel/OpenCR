/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef DELTA_H_
#define DELTA_H_

#include "Delta_Dynamixel.h"
#include "Delta_Drawing.h"
#include "Delta_Kinematics.h"

#define NUM_OF_JOINT 4
#define DXL_SIZE 5

#define DRAWING_LINE "drawing_line"
#define DRAWING_CIRCLE "drawing_circle"
#define DRAWING_RHOMBUS "drawing_rhombus"
#define DRAWING_HEART "drawing_heart"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL "tool_dxl"

#define CONTROL_TIME 0.010 //s

#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)


class Delta : public ROBOTIS_MANIPULATOR::RobotisManipulator
{
private:
  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::JointActuator *actuator_;
  ROBOTIS_MANIPULATOR::ToolActuator *tool_;

  DELTA_DRAWING::Line line_;
  DELTA_DRAWING::Circle circle_;
  DELTA_DRAWING::Rhombus rhombus_;
  DELTA_DRAWING::Heart heart_;

  bool platform_;
  std::vector<uint8_t> jointDxlId;
 public:
  Delta();
  virtual ~Delta();

  void initManipulator(bool using_platform, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000");
  void Process(double present_time);
  bool getPlatformFlag();
};

#endif // Delta_H_




