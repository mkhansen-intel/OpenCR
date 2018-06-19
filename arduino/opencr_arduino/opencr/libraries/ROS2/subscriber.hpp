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

#define DEFAULT_READER_XML ("<profiles><subscriber profile_name=\"default_xrce_subscriber_profile\"><topic><kind>NO_KEY</kind><name>%sTopic</name><dataType>%s</dataType><historyQos><kind>KEEP_LAST</kind><depth>10</depth></historyQos><durability><kind>TRANSIENT_LOCAL</kind></durability></topic></subscriber></profiles>")


namespace ros2
{

class Node;

template <typename MsgT>
class Subscriber
{

  public:
    Subscriber(micrortps::Participant_t* node, char* subscriber_profile)
    {
      MsgT topic;
      node_ = node;  
      subscriber_.is_init = false;

      char reader_profile[512] = {0, };
      sprintf(reader_profile, DEFAULT_READER_XML, topic.name_, topic.name_);
      is_registered_ = micrortps::createSubscriber(node_, &subscriber_, topic.id_, subscriber_profile, reader_profile);
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
    micrortps::Participant_t* node_;
    micrortps::Subscriber_t subscriber_;
};

} // namespace ros2

#endif /* ROS2_SUBSCRIBER_HPP_ */
