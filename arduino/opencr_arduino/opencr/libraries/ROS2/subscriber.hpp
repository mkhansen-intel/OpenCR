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
    Subscriber(micrortps::Participant_t* node, SubCallback cb)
    {
      node_ = node;  
      is_registered_ = micrortps::createSubscriber(node_, &subscriber_, topic_.reader_profile_);
      topic_.userCallback = cb;
    }

    void subscribe()
    {
      micrortps::subscribe(&subscriber_, topic_.deserialize, topic_.callback, NULL);
    }

    void recreate()
    {
      is_registered_ = micrortps::createSubscriber(node_, &subscriber_, topic_->reader_profile_);
    };  

    bool is_registered_;

  private:
    MsgT topic_;
    micrortps::Participant_t* node_;
    micrortps::Subscriber_t subscriber_;
};

} // namespace ros2

#endif /* ROS2_SUBSCRIBER_HPP_ */
