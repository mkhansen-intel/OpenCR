/*
 * ros2.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: kei
 */


#include "ros2.hpp"


static bool g_is_rmw_init = false;

bool ros2::init(OnTopic callback)
{
  g_is_rmw_init = micrortps::setup(callback);

  return g_is_rmw_init;
}

void ros2::spin(ros2::Node *node)
{
  // uint8_t pub_cnt = 0;
  // ros2::NodeHandle *pub;

  // while(pub_cnt < 20)
  // {
  //   pub = node->pub_list_[pub_cnt++];

  //   if(pub != NULL)
  //   {
  //     if(millis() - pub->last_call_time > pub->callback_interval && node->pub_list_[pub_cnt]->callback_interval > 0)
  //     {
  //       pub->timerCallback();
  //     }
  //   }
  // }
  node->callback();

  micrortps::runCommunication();
}
