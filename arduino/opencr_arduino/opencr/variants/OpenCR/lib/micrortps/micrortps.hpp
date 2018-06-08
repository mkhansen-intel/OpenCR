/*
 * ros2.hpp
 *
 *  Created on: May 16, 2018
 *      Author: Kei
 */

#ifndef MICRORTPS_HPP_
#define MICRORTPS_HPP_


#include "microcdr/microcdr.h"
#include "micrortps/client/client.h"
#include "micrortps/client/xrce_protocol_spec.h"



namespace micrortps {



typedef struct Subscriber{
  bool is_init;
  XRCEInfo info;
  XRCEInfo reader_info;
  XRCEInfo read_callback_info;
} Subscriber_t;

typedef struct Publisher{
  bool is_init;
  XRCEInfo info;
  XRCEInfo writer_info;
} Publisher_t;

typedef struct Participant{
  bool is_init;
  XRCEInfo info;
} Participant_t;


bool setup(void);
bool createParticipant(Participant_t* participant);
bool registerTopic(Participant_t* participant, char* topic_profile);
bool createPublisher(Participant_t* participant, Publisher_t* publisher, char* writer_profile);
bool createSubscriber(Participant_t* participant, Subscriber_t* subscriber, char* reader_profile);
void publish(Publisher_t* publisher, SerializeTopic func_serialize, void* topic);
void subscribe(Subscriber_t* subscriber, DeserializeTopic func_deserialize, OnTopicReceived callback_func, void* callback_func_args);
void listenFromAgent(void);



} /* namespace micrortps */



#endif /* ROS2_HPP_ */
