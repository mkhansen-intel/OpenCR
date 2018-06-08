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
  Publisher(micrortps::Participant_t* node)
  {
    node_ = node;
    MsgT topic;
    is_registered_ = micrortps::createPublisher(node_, &publisher_, topic.writer_profile_);
  }

  void publish(MsgT * topic)
  {
    micrortps::publish(&publisher_, topic->serialize, (void*) &topic->message_);
  }

  void recreate()
  {
    MsgT topic;
    is_registered_ = micrortps::createPublisher(node_, &publisher_, topic->writer_profile_);
  }

  bool is_registered_;

private:
  micrortps::Participant_t* node_;
  micrortps::Publisher_t publisher_;
  
};


} // namespace ros2



#endif /* ROS2_PUBLISHER_HPP_ */
