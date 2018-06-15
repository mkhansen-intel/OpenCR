/*
 * subscriber.hpp
 *
 *  Created on: May 16, 2018
 *      Author: Kei
 */

#ifndef ROS2_SUBSCRIBER_HPP_
#define ROS2_SUBSCRIBER_HPP_

#include "micrortps.hpp"
#include "topic.hpp"


namespace ros2
{

class Node;

template <typename MsgT>
class Subscriber
{

  public:
    Subscriber(micrortps::Participant_t* node, char* subscriber_profile)
    {
      node_ = node;  
      is_registered_ = micrortps::createSubscriber(node_, &subscriber_, topic_.id, subscriber_profile, topic_.reader_profile_);
    }

    void subscribe(uint8_t stream_id)
    {
      micrortps::subscribe(&subscriber_, stream_id);
    }

    void recreate()
    {
      //is_registered_ = micrortps::createSubscriber(node_, &subscriber_, topic_->reader_profile_);
    };  

    bool is_registered_;

  private:
    MsgT topic_;
    micrortps::Participant_t* node_;
    micrortps::Subscriber_t subscriber_;
};

} // namespace ros2

#endif /* ROS2_SUBSCRIBER_HPP_ */
