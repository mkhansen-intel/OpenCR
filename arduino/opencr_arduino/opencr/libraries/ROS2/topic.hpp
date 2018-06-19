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

#define DEFAULT_TOPIC_XML ("<dds><topic><name>%sTopic</name><dataType>%s</dataType></topic></dds>")

namespace ros2 {

/* Base Message Type */
template <typename MsgT>
class Topic
{

public:

  Topic(const char* name, uint8_t id):
    name_(name),
    id_(id)
  {   
  }

  virtual bool serialize(MicroBuffer* writer, const MsgT* topic) = 0;
  virtual bool deserialize(MicroBuffer* reader, MsgT* topic) = 0;
  virtual bool write(Session* session, ObjectId datawriter_id, StreamId stream_id, MsgT* topic) = 0;

  const char* name_;
  uint8_t id_;
};



} // namespace ros2



#endif /* ROS2_TOPIC_HPP_ */
