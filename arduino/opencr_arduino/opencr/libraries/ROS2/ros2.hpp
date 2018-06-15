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
    uint8_t err_code;

    Node()
    {
      err_code = 0;
      participant_.is_init = false;
      node_register_state_ = micrortps::createParticipant(&this->participant_);
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
        err_code = 1;
        return NULL;
      }

      // Register Topic
      ret = this->registerTopic<MsgT>();

      if (ret == false)
      {
        err_code = 2;
        return NULL;
      }

      sprintf(pub_profile, "<publisher name=\"%s\"", name);
      p_pub = new ros2::Publisher<MsgT>(&this->participant_, pub_profile);

      if(p_pub->is_registered_ == false)
      {
        err_code = 3;
        return NULL;
      }

      err_code = 0;

      return p_pub;
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
        err_code = 10 + 1;
        return NULL;
      }

      // Register Topic
      ret = this->registerTopic<MsgT>();

      if (ret == false)
      {
        err_code = 10 + 2;
        return NULL;
      }

      sprintf(sub_profile, "<subscriber name=\"%s\"", name);
      p_sub = new ros2::Subscriber<MsgT>(&this->participant_, sub_profile);

      if(p_sub->is_registered_ == false)
      {
        err_code = 10 + 3;
        return NULL;
      }

      err_code = 0;

      return p_sub;
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

bool init(OnTopic callback)
{
  g_is_rmw_init = micrortps::setup(callback);

  return g_is_rmw_init;
}

void spin()
{
  micrortps::runCommunication();
}


} /* namespace ros2 */



#endif /* ROS2_HPP_ */
