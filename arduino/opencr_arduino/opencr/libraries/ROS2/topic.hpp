/*
 * topic.hpp
 *
 *  Created on: May 16, 2018
 *      Author: Kei
 */

#ifndef ROS2_TOPIC_HPP_
#define ROS2_TOPIC_HPP_

#include <stdlib.h>
#include "micrortps.hpp"

namespace ros2 {

typedef void (*SubCallback)(const void* vtopic);

/* Base Message Type */
class Topic
{

public:
  Topic() : 
    profile_((char*)""),
    serialize(NULL),
    deserialize(NULL),
    userCallback(NULL),
    writer_profile_((char*)""),
    reader_profile_((char*)"")
  {};

  char *profile_;
  SerializeTopic serialize;
  DeserializeTopic deserialize;
  SubCallback userCallback;
  char *writer_profile_;
  char *reader_profile_;

  virtual void callback(XRCEInfo info, const void* vtopic, void* args) = 0;
};



} // namespace ros2



#endif /* ROS2_TOPIC_HPP_ */
