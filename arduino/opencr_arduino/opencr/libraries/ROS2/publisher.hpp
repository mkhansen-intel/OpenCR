/*
 * publisher.hpp
 *
 *  Created on: May 16, 2018
 *      Author: Kei
 */

#ifndef ROS2_PUBLISHER_HPP_
#define ROS2_PUBLISHER_HPP_

#include "micrortps.hpp"
#include "topic.hpp"

#define DEFAULT_WRITER_XML ("<profiles><publisher profile_name=\"default_xrce_publisher_profile\"><topic><kind>NO_KEY</kind><name>%sTopic</name><dataType>%s</dataType><historyQos><kind>KEEP_LAST</kind><depth>10</depth></historyQos><durability><kind>TRANSIENT_LOCAL</kind></durability></topic></publisher></profiles>")


namespace ros2 {

template <typename MsgT>
class Publisher
{
 
public:
  Publisher(micrortps::Participant_t* node, char* publisher_profile)
  {
    node_ = node;
    MsgT topic;
    publisher_.is_init = false;

    char writer_profile[512] = {0, };
    sprintf(writer_profile, DEFAULT_WRITER_XML, topic.name_, topic.name_);
    is_registered_ = micrortps::createPublisher(node_, &publisher_, publisher_profile, writer_profile);
  }

  void publish(MsgT * topic, StreamId stream_id)
  {
    if(publisher_.is_init ==  false)
    {
      return;
    }

    topic->write(node_->session, publisher_.writer_id, stream_id, topic);
  }

  void recreate()
  {
//    MsgT topic;
//    is_registered_ = micrortps::createPublisher(node_, &publisher_, topic->writer_profile_);
  }

  bool is_registered_;

private:
  micrortps::Participant_t* node_;
  micrortps::Publisher_t publisher_;
  
};


} // namespace ros2



#endif /* ROS2_PUBLISHER_HPP_ */
