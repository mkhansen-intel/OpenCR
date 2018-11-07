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

#include "../../include/open_manipulator_lib/om_dynamixel.h"

using namespace OM_DYNAMIXEL;

void JointDynamixel::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  String *get_arg_ = (String *)arg;

  bool result = JointDynamixel::initialize(actuator_id ,get_arg_[0]);

  if (result == false)   
    return;
}

void JointDynamixel::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  bool result = false;
  String *get_arg_ = (String *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = JointDynamixel::setOperatingMode(actuator_id, get_arg_[0]);
    result = JointDynamixel::setSDKHandler(actuator_id.at(0));
    if (result == false)
      return;
  }
  else
  {
    result = JointDynamixel::writeProfileValue(actuator_id, get_arg_[0], get_arg_[1].toInt());
    if (result == false)
      return;
  }
}

std::vector<uint8_t> JointDynamixel::getId()
{
  return dynamixel_.id;
}

void JointDynamixel::enable()
{
  for (uint32_t index = 0; index < dynamixel_.num; index++)
    dynamixel_workbench_->itemWrite(dynamixel_.id.at(index), "Torque_Enable", true);
}

void JointDynamixel::disable()
{
  for (uint32_t index = 0; index < dynamixel_.num; index++)
    dynamixel_workbench_->itemWrite(dynamixel_.id.at(index), "Torque_Enable", false);
}

bool JointDynamixel::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector)
{
  bool result = false;
  std::vector<double> radian_vector;

  for(int index = 0; index < value_vector.size(); index++)
  {
    radian_vector.push_back(value_vector.at(index).value);
  }

  result = JointDynamixel::writeGoalPosition(actuator_id, radian_vector);
  if (result == false)
    return false;

  return true;
}

std::vector<ROBOTIS_MANIPULATOR::Actuator> JointDynamixel::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  return JointDynamixel::receiveAllDynamixelValue(actuator_id);
}

//////////////////////////////////////////////////////////////////////////

bool JointDynamixel::initialize(std::vector<uint8_t> actuator_id, String dxl_baud_rate)
{
  bool result = false;

  dynamixel_.id = actuator_id;
  dynamixel_.num = actuator_id.size();
 
  dynamixel_workbench_ = new DynamixelWorkbench;
  
  result = dynamixel_workbench_->begin(DEVICE_NAME, dxl_baud_rate.toInt());
  if (result == false)
    return false;

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    result = dynamixel_workbench_->ping(id, &get_model_number);
    if (result == false)
    {
      //LOG::INFO("Please check your Dynamixel ID\n");
      return false;
    }
    else
    {
      //LOG::INFO("ID : " + String(dxl_id_.at(index)));
    }
  }

  return true;
}

bool JointDynamixel::setOperatingMode(std::vector<uint8_t> actuator_id, String dynamixel_mode)
{
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t effort = 0;
  const uint32_t current = 0;

  if (dynamixel_mode == "position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, effort);
      if (result == false)
      {
        //LOG::INFO("Failed to set the dxl control mode \n");
        return false;
      }
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->currentMode(actuator_id.at(num), current);
      if (result == false)
      {
        //LOG::INFO("Failed to set the dxl control mode \n");
        return false;
      }
    }
  }
  else
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, effort);
      if (result == false)
      {
        //LOG::INFO("Failed to set the dxl control mode \n");
        return false;
      }
    }
  }

  return true;
}

bool JointDynamixel::setSDKHandler(uint8_t actuator_id)
{
  dynamixel_workbench_->addSyncWrite("Goal_Position");
  dynamixel_workbench_->addSyncRead("Present_Position");
  return true;
}

bool JointDynamixel::writeProfileValue(std::vector<uint8_t> actuator_id, String profile_mode, uint32_t value)
{
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();
  
  for (uint8_t num = 0; num < actuator_id.size(); num++)
  {
    result = dynamixel_workbench_->itemWrite(actuator_id.at(num), char_profile_mode, value);
    if (result == false)
    {
      //LOG::INFO("Failed to write the dxl profile\n");
      return false;
    }
  }

  return true;
}

bool JointDynamixel::writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector)
{
  bool result = false;

  uint8_t id_array[actuator_id.size()];
  int32_t goal_position[actuator_id.size()];

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    id_array[index] = actuator_id.at(index);
    goal_position[index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), radian_vector.at(index));
  }

  result = dynamixel_workbench_->syncWrite(&id_array[0], actuator_id.size(), "Goal_Position", goal_position);

  if (result == false)
  {
    //LOG::INFO("Failed to write the dxl goal position\n");
    return false;
  }

  return true;
}

std::vector<ROBOTIS_MANIPULATOR::Actuator> JointDynamixel::receiveAllDynamixelValue(std::vector<uint8_t> actuator_id)
{
  ROBOTIS_MANIPULATOR::Actuator value;
  std::vector<ROBOTIS_MANIPULATOR::Actuator> all_values;

  int32_t* get_value;

  if (dynamixel_workbench_->getProtocolVersion() == 2.0)
  {
    get_value = dynamixel_workbench_->syncRead("Present_Position");    
  }
  else
  {
    for (uint8_t index = 0; index < actuator_id.size(); index++)
      get_value[index] = dynamixel_workbench_->itemRead(actuator_id.at(index), "Present_Position");
  }
 
  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    int32_t position = get_value[index];
    int32_t velocity = 0;
    int32_t current = 0;

    value.value    = dynamixel_workbench_->convertValue2Radian(actuator_id.at(index), position);
    value.velocity = velocity;
    //value.current = current;

    all_values.push_back(value);
  }
  return all_values;
}

//////////////////////////////////////tool actuator

void GripperDynamixel::init(uint8_t actuator_id, const void *arg)
{
  String *get_arg_ = (String *)arg;

  bool result = GripperDynamixel::initialize(actuator_id ,get_arg_[0]);

  if (result == false)
    return;
}

void GripperDynamixel::setMode(const void *arg)
{
  bool result = false;
  const char* log = NULL;

  String *get_arg_ = (String *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = GripperDynamixel::setOperatingMode(get_arg_[0]);
    result = GripperDynamixel::setSDKHandler();
    if (result == false)
      return;
  }
  else
  {
    result = GripperDynamixel::writeProfileValue(get_arg_[0], get_arg_[1].toInt());
    if (result == false)
      return;
  }
}

uint8_t GripperDynamixel::getId()
{
  return dynamixel_.id.at(0);
}

void GripperDynamixel::enable()
{
  dynamixel_workbench_->itemWrite(dynamixel_.id.at(0), "Torque_Enable", true);
}

void GripperDynamixel::disable()
{
  dynamixel_workbench_->itemWrite(dynamixel_.id.at(0), "Torque_Enable", false);
}

bool GripperDynamixel::sendToolActuatorValue(double value)
{
  return GripperDynamixel::writeGoalPosition(value);
}

double GripperDynamixel::receiveToolActuatorValue()
{
  return GripperDynamixel::receiveDynamixelValue();
}

//////////////////////////////////////////////////////////////////////////

bool GripperDynamixel::initialize(uint8_t actuator_id, String dxl_baud_rate)
{
  bool result = false;

  dynamixel_.id.push_back(actuator_id);
  dynamixel_.num = 1;

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->begin(DEVICE_NAME, dxl_baud_rate.toInt());
  if (result == false)
    return false;

  uint16_t get_model_number;
  result = dynamixel_workbench_->ping(dynamixel_.id.at(0), &get_model_number);

  if (result == false)
  {
    //LOG::INFO("Please check your Dynamixel ID\n");
    return false;
  }

  return true;
}

bool GripperDynamixel::setOperatingMode(String dynamixel_mode)
{
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t effort = 0;
  const uint32_t current = 100;

  if (dynamixel_mode == "position_mode")
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, effort);
    if (result == false)
    {
      //LOG::INFO("Failed to set the dxl control mode \n");
      return false;
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    result = dynamixel_workbench_->currentMode(dynamixel_.id.at(0), current);
    if (result == false)
    {
      //LOG::INFO("Failed to set the dxl control mode \n");
      return false;
    }
  }
  else
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, effort);
    if (result == false)
    {
      //LOG::INFO("Failed to set the dxl control mode \n");
      return false;
    }
  }

  return true;
}

bool GripperDynamixel::writeProfileValue(String profile_mode, uint32_t value)
{
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();
  
  result = dynamixel_workbench_->itemWrite(dynamixel_.id.at(0), char_profile_mode, value);
  if (result == false)
  {
    //LOG::INFO("Failed to write the dxl profile\n");
    return false;
  }
  return true;
}

bool GripperDynamixel::setSDKHandler()
{
  dynamixel_workbench_->addSyncWrite("Goal_Position");
  dynamixel_workbench_->addSyncRead("Present_Position");
  return true;
}

bool GripperDynamixel::writeGoalPosition(double radian)
{
  bool result = false;

  int32_t goal_position = 0;;

  goal_position = dynamixel_workbench_->convertRadian2Value(dynamixel_.id.at(0), radian);

  result = dynamixel_workbench_->syncWrite("Goal_Position", &goal_position);
  
  if (result == false)
  {
    //LOG::INFO("Failed to write the dxl goal position\n");
    return false;
  }
  return true;
}

double GripperDynamixel::receiveDynamixelValue()
{
  uint8_t id_array[1] = {dynamixel_.id.at(0)};
  int32_t* get_value;

  if (dynamixel_workbench_->getProtocolVersion() == 2.0)
  {
    get_value = dynamixel_workbench_->syncRead("Present_Position");    
  }
  else
  {
    get_value[0] = dynamixel_workbench_->itemRead(dynamixel_.id.at(0), "Present_Position");
  }
  return dynamixel_workbench_->convertValue2Radian(dynamixel_.id.at(0), get_value[0]);
}
