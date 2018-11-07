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

#ifndef OM_LINK_SUCTION_H_
#define OM_LINK_SUCTION_H_

#include <robotis_manipulator.h>
#include <robotis_manipulator_common.h>
#include "Arduino.h"
// #include <iostream>
// #include <cstdio>

class SuctionModule : public ROBOTIS_MANIPULATOR::ToolActuator
{
 private:
  uint8_t relay_pin_;
  int8_t id_;
  bool suction_flag_;

 public:
  SuctionModule() {}
  virtual ~SuctionModule() {}

  virtual void init(uint8_t actuator_id, const void *arg);
  virtual void setMode(const void *arg);
  virtual uint8_t getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendToolActuatorValue(double value);
  virtual double receiveToolActuatorValue();

////////////////////////////////////////////////////////////////

  void suctionInit();
  void suctionOn();
  void suctionOff();
};




#endif //OM_LINK_SUCTION_H_


