/*
 * ros2.hpp
 *
 *  Created on: May 16, 2018
 *      Author: Kei
 */

#ifndef ROS2_HPP_
#define ROS2_HPP_

#include "publisher.hpp"
#include "subscriber.hpp"


namespace ros2 {

class Node
{
  public:
    Node()
    {
      node_register_state_ = micrortps::createParticipant(&this->participant_, NULL);
    }

    template <
      typename MsgT>
    Publisher<MsgT>* createPublisher(const char* name)
    {
      bool ret;
      char pub_profile[100];
      ros2::Publisher<MsgT> *p_pub = NULL;

      if(this->node_register_state_ == false)
      {
        return NULL;
      }

      // Register Topic
      ret = this->registerTopic<MsgT>();

      if (ret == true)
      {
        sprintf(pub_profile, "<publisher name=\"%s\"", name);
        p_pub = new ros2::Publisher<MsgT>(&this->participant_, pub_profile);
      }

      return p_pub->is_registered_ == true ? p_pub : NULL;
    }


    template <
      typename MsgT>
    Subscriber<MsgT>* createSubscriber(const char* name)
    {
      bool ret;
      char sub_profile[100];
      ros2::Subscriber<MsgT> *p_sub = NULL;
      
      if(this->node_register_state_ == false)
      {
        return NULL;
      }

      // Register Topic
      ret = this->registerTopic<MsgT>();

      if (ret == true)
      {
        sprintf(sub_profile, "<subscriber name=\"%s\"", name);
        p_sub = new ros2::Subscriber<MsgT>(&this->participant_, sub_profile);
      }

      return p_sub->is_registered_ == true ? p_sub : NULL;
    }


  private:
    bool node_register_state_;
    micrortps::Participant_t participant_;

    template <
      typename MsgT>
    bool registerTopic()
    {
      bool ret;
      MsgT topic;

      if(this->node_register_state_ == false)
      {
        return false;
      }

      ret = micrortps::registerTopic(&this->participant_, topic.profile_);

      return ret;
    }
};



static bool g_is_rmw_init = false;

void init()
{
  g_is_rmw_init = false;
}

void spin()
{
  micrortps::runCommunication();
}


} /* namespace ros2 */



#endif /* ROS2_HPP_ */
