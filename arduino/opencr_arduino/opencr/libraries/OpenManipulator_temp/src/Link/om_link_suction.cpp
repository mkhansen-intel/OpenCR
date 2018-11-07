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

#include "../../include/Link/om_link_suction.h"


void SuctionModule::init(uint8_t actuator_id, const void *arg)
{
  id_ = actuator_id;
   
  uint8_t *get_arg = (uint8_t *)arg;
  relay_pin_ = get_arg[0];

  suctionInit();
}

void SuctionModule::setMode(const void *arg){}

uint8_t SuctionModule::getId()
{
  return id_;
}

void SuctionModule::enable(){}

void SuctionModule::disable(){}

bool SuctionModule::sendToolActuatorValue(double value)
{
  if(value > 0)
  {
    suction_flag_ = true;
    suctionOn();
  }  
  else
  {
    suction_flag_ = false;
    suctionOff();
  }
}

double SuctionModule::receiveToolActuatorValue()
{
  if(suction_flag_)
  {
    return 1.0;
  }
  else
  {
    return -1.0;
  }
}

void SuctionModule::suctionInit()
{
  pinMode(relay_pin_, OUTPUT);
}

void SuctionModule::suctionOn()
{
  digitalWrite(relay_pin_, HIGH);
}

void SuctionModule::suctionOff()
{
  digitalWrite(relay_pin_, LOW);
}
