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


namespace ros2 {

template <typename MsgT>
class Publisher
{
 
public:
  Publisher(micrortps::Participant_t* node, char* publisher_profile)
  {
    node_ = node;
    MsgT topic;
    is_registered_ = micrortps::createPublisher(node_, &publisher_, publisher_profile, topic.writer_profile_);
  }

  void publish(MsgT * topic, StreamId stream_id)
  {
    topic->write(node_->session, publisher_.writer_id, stream_id, &topic->message_);
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
