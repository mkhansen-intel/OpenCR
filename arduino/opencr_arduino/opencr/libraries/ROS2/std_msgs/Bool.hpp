// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _STD_MSGS_BOOL_HPP_
#define _STD_MSGS_BOOL_HPP_


#include "micrortps.hpp"
#include <topic_config.h>
#include <topic.hpp>

#ifdef STD_MSGS_BOOL_TOPIC

namespace std_msgs {


class Bool : public ros2::Topic<Bool>
{
public:
  typedef bool _data_type;
  _data_type data;

  Bool():
    data(false)
  { 
    name_ = (char*) "Bool";
    id_ = STD_MSGS_BOOL_TOPIC;
  }

  virtual bool serialize(MicroBuffer* writer, const Bool* topic)
  {
      serialize_bool(writer, topic->data);

      return writer->error == BUFFER_OK;
  }

  virtual bool deserialize(MicroBuffer* reader, Bool* topic)
  {
      deserialize_bool(reader, &topic->data);

      return reader->error == BUFFER_OK;
  }

  virtual bool write(Session* session, ObjectId datawriter_id, StreamId stream_id, Bool* topic)
  {
      if (session == NULL)
      {
          return false;
      }

      bool result = false;
      uint32_t topic_size = size_of_topic(topic);
      MicroBuffer* topic_buffer = NULL;

      if (128 < stream_id)
      {
          topic_buffer = prepare_best_effort_stream_for_topic(&session->output_best_effort_stream, datawriter_id, topic_size);
      }
      else
      {
          topic_buffer = prepare_reliable_stream_for_topic(&session->output_reliable_stream, datawriter_id, topic_size);
      }

      if (topic_buffer != NULL)
      {
          result = serialize(topic_buffer, topic);
      }

      return result;
  }

private :

  uint32_t size_of_topic(const Bool* topic)
  {
      (void)(topic);
      uint32_t size = 0;

      size += 1 + get_alignment(size, 1);

      return size;
  }

};

} // namespace std_msgs

#endif // STD_MSGS_BOOL_TOPIC

#endif // _STD_MSGS_BOOL_HPP_
