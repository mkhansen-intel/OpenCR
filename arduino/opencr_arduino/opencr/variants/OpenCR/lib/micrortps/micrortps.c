/*
 * micrortps.c
 *
 *  Created on: Mar 20, 2018
 *      Author: opus
 */

#include <stdlib.h>
#include <string.h>

#include "micrortps.h"

#include "hw.h"

#define RTPS_BUFFER_SIZE     4096
#define RTPS_NODE_MAX        5

//-- Internal Variables
//
typedef struct
{
  RtpsResponse_t create;
  RtpsResponse_t update;
  RtpsResponse_t delete;
  RtpsResponse_t lookup;
  RtpsResponse_t read;
  RtpsResponse_t write;
} RtpsResponseTable_t;

bool is_rtps_init_done = false;
ClientState    *rtps_client;
XRCEInfo        client_info;
RtpsResponseTable_t ResponseTable;

static RtpsNode_t RtpsNodes[RTPS_NODE_MAX] = {0, };

//-- External Variables
//

//-- Internal Functions
//
void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args);
bool uRtpsCheckResponse(uint16_t object_id, RtpsProcess_t process, uint32_t timeout);
bool uRtpsRegisterTopic(RtpsNode_t* p_node, TopicInfo_t* p_topic, uint32_t timeout);


//-- External Functions
//


bool uRtpsInit(void)
{
  return true;
}

bool uRtpsSetup(void)
{
  bool ret;

  if(uRtpsIsInit() == true)
  {
    return true;
  }

  if(rtps_client == NULL)
  {
    rtps_client = new_serial_client_state(RTPS_BUFFER_SIZE, "opencr_usb");
  }

  client_info = create_client(rtps_client, on_status_received, &ResponseTable);
  send_to_agent(rtps_client);

  ret = uRtpsCheckResponse(client_info.object_id, CREATE_CLIENT, 500);

  if(ret == true)
  {
    is_rtps_init_done = true;
  }

  return is_rtps_init_done;
}

bool uRtpsIsInit(void)
{
  return is_rtps_init_done;
}

void uRtpsStop(void)
{
  if(rtps_client != NULL)
  {
    free_client_state(rtps_client);
  }
}


RtpsNode_t* uRtpsCreateNode(void)
{
  uint32_t node_num;
  RtpsNode_t* p_node = NULL;

  if(uRtpsIsInit() == false)
  {
    if(uRtpsSetup() == false)
    {
      return NULL;
    }
  }

  // Check node which be usable.
  for(node_num = 0; node_num < RTPS_NODE_MAX; node_num++)
  {
    if(RtpsNodes[node_num].is_init == false)
    {
      p_node = &RtpsNodes[node_num];
      break;
    }
  }

  // Is using all nodes?
  if(node_num == RTPS_NODE_MAX)
  {
    return p_node;
  }

  // Create Participant
  for(uint32_t i = 0; i < NODE_MAX_TOPICS; i++)
  {
    p_node->topic_profile[i] = NULL;
  }

  p_node->participant_info = create_participant(rtps_client);
  send_to_agent(rtps_client);

  p_node->is_init = uRtpsCheckResponse(p_node->participant_info.object_id, CREATE_PARTICIPANT, 500);


  return p_node->is_init == true ? p_node : NULL;
}

bool uRtpsRegisterTopic(RtpsNode_t* p_node, TopicInfo_t* p_topic, uint32_t timeout)
{
  bool ret;
  uint32_t idx;
  string profile_xml;

  for(idx = 0; idx < NODE_MAX_TOPICS; idx++)
  {
    //check already registered or not.
    if(strcmp(p_node->topic_profile[idx], p_topic->profile) == 0)
    {
      return true;
    }

    if(p_node->topic_profile[idx] == NULL)
    {
      break;
    }
  }

  if(idx == NODE_MAX_TOPICS)
  {
    //return false;
  }

  // Create Topic
  profile_xml.data = p_topic->profile;
  profile_xml.length = strlen(p_topic->profile);
  XRCEInfo topic_info = create_topic(rtps_client, p_node->participant_info.object_id, profile_xml);
  p_node->topic_profile[idx] = p_topic->profile;
  send_to_agent(rtps_client);

  ret = uRtpsCheckResponse(topic_info.object_id, CREATE_TOPIC, timeout);

  return ret;
}

RtpsPublisher_t* uRtpsCreatePub(RtpsNode_t* p_node, TopicInfo_t* p_topic, const char* data_writer_profile, uint32_t timeout)
{
  bool ret;
  uint8_t idx;
  RtpsPublisher_t* p_pub = NULL;
  string profile_xml;

  if(p_node->is_init == false)
  {
    return NULL;
  }

  for(idx = 0; idx < NODE_MAX_PUBLISHERS; idx++)
  {
    if(p_node->publishers[idx].is_init == false)
    {
      p_pub = &p_node->publishers[idx];
      break;
    }
  }

  if(p_pub == NULL)
  {
    return NULL;
  }

  // Create Topic
  ret = uRtpsRegisterTopic(p_node, p_topic, timeout);

  if (ret == true)
  {
    // Create Publisher
    p_pub->publisher_info = create_publisher(rtps_client, p_node->participant_info.object_id);
    send_to_agent(rtps_client);

    ret = uRtpsCheckResponse(p_pub->publisher_info.object_id, CREATE_PUBLISHER,
        timeout);

    if (ret == true)
    {
      // Create Writer
      profile_xml.data = data_writer_profile;
      profile_xml.length = strlen(data_writer_profile);
      p_pub->data_writer_info = create_data_writer(rtps_client,
          p_node->participant_info.object_id, p_pub->publisher_info.object_id,
          profile_xml);
      send_to_agent(rtps_client);

      ret = uRtpsCheckResponse(p_pub->data_writer_info.object_id, CREATE_WRITER,
          timeout);

      if (ret == true)
      {
        p_pub->func_serialize = p_topic->serialize_func;
        p_pub->is_init = true;
      }
    }
  }

  return p_pub->is_init == true ? p_pub : NULL;
}

RtpsSubscriber_t* uRtpsCreateSub(RtpsNode_t* p_node, TopicInfo_t* p_topic, const char* data_reader_profile, uint32_t timeout)
{
  bool ret;
  uint8_t idx;
  RtpsSubscriber_t* p_sub = NULL;
  string profile_xml;

  if(p_node->is_init == false)
  {
    return NULL;
  }

  for(idx = 0; idx < NODE_MAX_SUBSCRIBERS; idx++)
  {
    if(p_node->subscribers[idx].is_init == false)
    {
      p_sub = &p_node->subscribers[idx];
      break;
    }
  }

  if(p_sub == NULL)
  {
    return NULL;
  }

  // Create Topic
  ret = uRtpsRegisterTopic(p_node, p_topic, timeout);

  if (ret == true)
  {
    // Create Subscriber
    p_sub->subscriber_info = create_subscriber(rtps_client, p_node->participant_info.object_id);
    send_to_agent(rtps_client);

    ret = uRtpsCheckResponse(p_sub->subscriber_info.object_id, CREATE_SUBSCRIBER,
        timeout);

    if (ret == true)
    {
      // Create Reader
      profile_xml.data = data_reader_profile;
      profile_xml.length = strlen(data_reader_profile);
      p_sub->data_reader_info = create_data_reader(rtps_client,
          p_node->participant_info.object_id, p_sub->subscriber_info.object_id,
          profile_xml);
      send_to_agent(rtps_client);

      ret = uRtpsCheckResponse(p_sub->data_reader_info.object_id, CREATE_READER,
          timeout);

      if (ret == true)
      {
        p_sub->func_deserialize = p_topic->deserialize_func;
        p_sub->is_init = true;
      }
    }
  }

  return p_sub->is_init == true ? p_sub : NULL;
}

void uRtpsWrite(RtpsPublisher_t* p_pub, void* topic)
{
  if(uRtpsIsInit() == false || p_pub == NULL)
  {
    return;
  }

  write_data(rtps_client, p_pub->data_writer_info.object_id, p_pub->func_serialize, topic);
  send_to_agent(rtps_client);
}

void uRtpsRead(RtpsSubscriber_t* p_sub, OnTopicReceived callback_func, void* callback_func_args)
{
  if(uRtpsIsInit() == false || p_sub == NULL)
  {
    return;
  }

  p_sub->read_callback_info = read_data(rtps_client, p_sub->data_reader_info.object_id, p_sub->func_deserialize, callback_func, callback_func_args);
  send_to_agent(rtps_client);
}

bool uRtpsCheckResponse(uint16_t object_id, RtpsProcess_t process, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time;

  pre_time = millis();
  while((millis() - pre_time < timeout))
  {
    receive_from_agent(rtps_client);

    switch(process)
    {
      case CREATE_CLIENT :
      case CREATE_PARTICIPANT:
      case CREATE_PUBLISHER:
      case CREATE_SUBSCRIBER:
      case CREATE_WRITER:
      case CREATE_READER:
        if(ResponseTable.create.ids.object_id == object_id && ResponseTable.create.status == STATUS_OK)
        {
          ret = true;
        }
        break;

      case CREATE_TOPIC:
        if(ResponseTable.create.ids.object_id == object_id)
        {
          ret = true;
        }
        break;
    }

    if(ret == true)
    {
      break;
    }
  }
  return ret;
}

void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args)
{
  RtpsResponseTable_t *tbl = (RtpsResponseTable_t *) args;
  RtpsResponse_t *recv;

  if(tbl == NULL)
  {
    return;
  }

  switch(operation)
  {
    case STATUS_LAST_OP_CREATE:
      recv = (RtpsResponse_t *) &tbl->create;
      break;
    case STATUS_LAST_OP_UPDATE:
      recv = (RtpsResponse_t *) &tbl->update;
      break;
    case STATUS_LAST_OP_DELETE:
      recv = (RtpsResponse_t *) &tbl->delete;
      break;
    case STATUS_LAST_OP_LOOKUP:
      recv = (RtpsResponse_t *) &tbl->lookup;
      break;
    case STATUS_LAST_OP_READ:
      recv = (RtpsResponse_t *) &tbl->read;
      break;
    case STATUS_LAST_OP_WRITE:
      recv = (RtpsResponse_t *) &tbl->write;
      break;
    default :
      return;
  }

  recv->ids = info;
  recv->operation = operation;
  recv->status = status;
}


void uRtpslistenToAgent(void)
{
  if(rtps_client != NULL)
  {
    receive_from_agent(rtps_client);
  }
}
