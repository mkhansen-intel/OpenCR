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

/* Base Message Type */
template <typename MsgT>
class Topic
{

public:
  Topic() : 
    serialize(NULL),
    deserialize(NULL),
    write(NULL),
    profile_((char*)""),
    writer_profile_((char*)""),
    reader_profile_((char*)"")
  {};

  typedef bool (*Serialize)(MicroBuffer* writer, const MsgT* topic);
  typedef bool (*Deserialize)(MicroBuffer* reader, MsgT* topic);
  typedef bool (*Write)(Session* session, ObjectId datawriter_id, StreamId stream_id, MsgT* topic);

  Serialize serialize; 
  Deserialize deserialize;
  Write write;

  uint8_t id;
  char *profile_;
  char *writer_profile_;
  char *reader_profile_;
};



} // namespace ros2



#endif /* ROS2_TOPIC_HPP_ */
