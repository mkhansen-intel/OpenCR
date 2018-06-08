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
      node_register_state_ = micrortps::createParticipant(&this->participant_);
    }


    template <
      typename MsgT>
    Publisher<MsgT>* createPublisher()
    {
      bool ret;
      ros2::Publisher<MsgT> *p_pub = NULL;

      if(this->node_register_state_ == false)
      {
        return NULL;
      }

      // Register Topic
      ret = ros2::Node::registerTopic<MsgT>();

      if (ret == true)
      {
        p_pub = new ros2::Publisher<MsgT>(&this->participant_);
      }

      return p_pub->is_registered_ == true ? p_pub : NULL;
    }


    template <
      typename MsgT>
    Subscriber<MsgT>* createSubscriber(SubCallback callback_fn)
    {
      bool ret;
      ros2::Subscriber<MsgT> *p_sub = NULL;
      
      if(this->node_register_state_ == false)
      {
        return NULL;
      }

      // Register Topic
      ret = this->registerTopic<MsgT>();

      if (ret == true)
      {
        p_sub = new ros2::Subscriber<MsgT>(&this->participant_, callback_fn);
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
      ros2::Topic* p_topic = (ros2::Topic*) &topic;

      if(this->node_register_state_ == false)
      {
        return false;
      }

      ret = micrortps::registerTopic(&this->participant_, p_topic->profile_);

      return ret;
    }
};



static bool g_is_rmw_init = false;

void init()
{
	if(g_is_rmw_init == true)
	{
		return;
	}

  g_is_rmw_init = micrortps::setup();
}

void spin(Node & node)
{
  ((void)(node));

  micrortps::listenFromAgent();
}


} /* namespace ros2 */



#endif /* ROS2_HPP_ */
