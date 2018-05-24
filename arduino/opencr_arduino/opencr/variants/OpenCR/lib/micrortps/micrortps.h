/*
 * micrortps.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Kei
 */

#ifndef MICRORTPS_H_
#define MICRORTPS_H_

#include "microcdr/microcdr.h"
#include "micrortps/client/client.h"
#include "micrortps/client/xrce_protocol_spec.h"

#include "topic_info.h"

#ifdef __cplusplus
 extern "C" {
#endif


#define NODE_MAX_PUBLISHERS   25
#define NODE_MAX_SUBSCRIBERS  25
#define NODE_MAX_TOPICS       (NODE_MAX_PUBLISHERS + NODE_MAX_SUBSCRIBERS)

typedef struct RtpsResponse{
  XRCEInfo ids;
  uint8_t  operation;
  uint8_t  status;
} RtpsResponse_t;

typedef struct RtpsSubscriber{
  bool is_init;
  XRCEInfo subscriber_info;
  XRCEInfo data_reader_info;
  XRCEInfo read_callback_info;
  DeserializeTopic func_deserialize;
} RtpsSubscriber_t;

typedef struct RtpsPublisher{
  bool is_init;
  XRCEInfo publisher_info;
  XRCEInfo data_writer_info;
  SerializeTopic func_serialize;
} RtpsPublisher_t;

typedef struct RtpsNode{
  bool is_init;
  bool is_registered;
  XRCEInfo participant_info;
  RtpsPublisher_t publishers[NODE_MAX_PUBLISHERS];
  RtpsSubscriber_t subscribers[NODE_MAX_SUBSCRIBERS];
  char* topic_profile[NODE_MAX_TOPICS];
} RtpsNode_t;

typedef enum {
  CREATE_CLIENT = 0,
  CREATE_PARTICIPANT,
  CREATE_TOPIC,
  CREATE_PUBLISHER,
  CREATE_SUBSCRIBER,
  CREATE_WRITER,
  CREATE_READER,
} RtpsProcess_t;


bool uRtpsIsInit(void);
bool uRtpsInit(void);
bool uRtpsSetup(void);
void uRtpsStop(void);
RtpsNode_t* uRtpsCreateNode(void);
RtpsPublisher_t* uRtpsCreatePub(RtpsNode_t* p_node, TopicInfo_t* p_topic, const char* data_writer_profile, uint32_t timeout);
RtpsSubscriber_t* uRtpsCreateSub(RtpsNode_t* p_node, TopicInfo_t* p_topic, const char* data_reader_profile, uint32_t timeout);
void uRtpsWrite(RtpsPublisher_t* p_pub, void* topic);
void uRtpsRead(RtpsSubscriber_t* p_sub, OnTopicReceived callback_func, void* callback_func_args);

void uRtpslistenToAgent(void);




#ifdef __cplusplus
 }
#endif

#endif /* MICRORTPS_H_ */
