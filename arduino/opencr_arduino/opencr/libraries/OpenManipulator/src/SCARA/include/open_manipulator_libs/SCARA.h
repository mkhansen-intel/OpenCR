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

#ifndef SCARA_H_
#define SCARA_H_

#include "SCARA_Dynamixel.h"
#include "SCARA_Drawing.h"
#include "SCARA_Kinematics.h"

#if defined(__OPENCR__)
  typedef String		  STRING;
#else
  typedef std::string STRING;
#endif

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define TOOL 4

#define NUM_OF_JOINT 3  // Number of joints <- not being used...
#define DXL_SIZE 4      // Number of dxls including dxls for tools

#define DRAWING_LINE 0
#define DRAWING_CIRCLE 1
#define DRAWING_RHOMBUS 2
#define DRAWING_HEART 3
#define DRAWING_CIRCLE2 4
#define DRAWING_RHOMBUS2 5
#define DRAWING_HEART2 6

#define JOINT_DYNAMIXEL 0
#define TOOL_DYNAMIXEL 1

#define CONTROL_TIME 0.010 // unit: s

#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)


class SCARA : public ROBOTIS_MANIPULATOR::RobotisManipulator
{
private:
  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::JointActuator *actuator_;
  ROBOTIS_MANIPULATOR::ToolActuator *tool_;

  SCARA_DRAWING::Line line_;
  SCARA_DRAWING::Circle circle_;
  SCARA_DRAWING::Rhombus rhombus_;
  SCARA_DRAWING::Heart heart_;

  bool using_platform_;
  bool processing_; // for what???

  std::vector<uint8_t> jointDxlId;

 public:
  SCARA();
  virtual ~SCARA();

  void initManipulator(bool using_platform, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000");
  void process(double present_time);  
  bool getPlatformFlag();
};

#endif // SCARA_H_




