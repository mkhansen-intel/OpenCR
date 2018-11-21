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

#ifndef SUCTION_H_
#define SUCTION_H_

#include "Delta.h"

void suctionInit()
{
#ifdef PLATFORM
  if(suction)
    pinMode(4, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(12, OUTPUT);
#endif
}

void suctionOn()
{
#ifdef PLATFORM
  if(suction)
    digitalWrite(RELAY_PIN, HIGH);
#endif
}

void suctionOff()
{
#ifdef PLATFORM
  if(suction)
    digitalWrite(RELAY_PIN, LOW);
#endif
}


#endif







