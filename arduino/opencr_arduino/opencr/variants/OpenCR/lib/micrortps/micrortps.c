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
#define RTPS_PUBLISHER_MAX   32
#define RTPS_SUBSCRIBER_MAX  32

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

uint8_t create_pub_cnt, create_sub_cnt;
RtpsPublisher_t  publisher_table[RTPS_PUBLISHER_MAX];
RtpsSubscriber_t subscriber_table[RTPS_SUBSCRIBER_MAX];

//-- External Variables
//

//-- Internal Functions
//
void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args);
bool microRtpsCheckRespose(uint16_t object_id, RtpsProcess_t process, uint32_t timeout);



//-- External Functions
//
bool microRtpsIsInit(void);
bool microRtpsInit(void);
bool microRtpsSetup(void);
RtpsPublisher_t* microRtpsCreatePub(RtpsXml topic_profile, RtpsXml data_writer_profile, SerializeTopic func_serialize, uint32_t timeout);
RtpsSubscriber_t* microRtpsCreateSub(RtpsXml topic_profile, RtpsXml data_reader_profile, DeserializeTopic func_deserialize, uint32_t timeout);
void microRtpsWrite(RtpsPublisher_t* p_pub, void* topic);
void microRtpsRead(RtpsSubscriber_t* p_sub, OnTopicReceived callback_func, void* callback_func_args);



bool microRtpsInit(void)
{
  return true;
}

bool microRtpsSetup(void)
{
  bool ret;

  if(microRtpsIsInit() == true)
  {
    return true;
  }

  if(rtps_client == NULL)
  {
    rtps_client = new_serial_client_state(RTPS_BUFFER_SIZE, "opencr_usb");
  }

  client_info = create_client(rtps_client, on_status_received, &ResponseTable);
  send_to_agent(rtps_client);

  ret = microRtpsCheckRespose(client_info.object_id, CREATE_CLIENT, 500);

  if(ret == true)
  {
    is_rtps_init_done = true;
  }

  return is_rtps_init_done;
}

bool microRtpsIsInit(void)
{
  return is_rtps_init_done;
}

void microRtpsStop(void)
{
  if(rtps_client != NULL)
  {
    free_client_state(rtps_client);
  }
}

RtpsPublisher_t* microRtpsCreatePub(RtpsXml topic_profile, RtpsXml data_writer_profile, SerializeTopic func_serialize, uint32_t timeout)
{
  bool ret;
  uint8_t idx;
  RtpsPublisher_t *p_pub;
  string profile_xml;

  if(microRtpsIsInit() == false)
  {
    if(microRtpsSetup() == false)
    {
      return NULL;
    }
  }

  for(idx = 0; idx < RTPS_PUBLISHER_MAX; idx++)
  {
    if(publisher_table[idx].is_init == false)
    {
      p_pub = &publisher_table[idx];
      break;
    }
  }
  if(idx == RTPS_PUBLISHER_MAX)
  {
    return NULL;
  }


  // Create Participant
  p_pub->participant_info = create_participant(rtps_client);
  send_to_agent(rtps_client);

  ret = microRtpsCheckRespose(p_pub->participant_info.object_id, CREATE_PARTICIPANT, 500);
  
  if(ret == true)
  {
    // Create Topic
    profile_xml.data = topic_profile;
    profile_xml.length = strlen(topic_profile);
    XRCEInfo topic_info = create_topic(rtps_client, p_pub->participant_info.object_id, profile_xml);
    send_to_agent(rtps_client);

    ret = microRtpsCheckRespose(topic_info.object_id, CREATE_TOPIC, timeout);

    if(ret == true)
    {
      // Create Publisher
      p_pub->publisher_info = create_publisher(rtps_client, p_pub->publisher_info.object_id);
      send_to_agent(rtps_client);

      ret = microRtpsCheckRespose(p_pub->publisher_info.object_id, CREATE_PUBLISHER, timeout);

      if(ret == true)
      {
        // Create Writer
        profile_xml.data = data_writer_profile;
        profile_xml.length = strlen(data_writer_profile);
        p_pub->data_writer_info = create_data_writer(rtps_client, p_pub->participant_info.object_id, p_pub->publisher_info.object_id, profile_xml);
        send_to_agent(rtps_client);

        ret = microRtpsCheckRespose(p_pub->data_writer_info.object_id, CREATE_WRITER, timeout);

        if(ret == true)
        {
          p_pub->func_serialize = func_serialize;
          p_pub->is_init = true;
        }
      }
    }
  }

  return p_pub->is_init == true ? p_pub : NULL;
}

RtpsSubscriber_t* microRtpsCreateSub(RtpsXml topic_profile, RtpsXml data_reader_profile, DeserializeTopic func_deserialize, uint32_t timeout)
{
  bool ret;
  uint8_t idx;
  RtpsSubscriber_t *p_sub;
  string profile_xml;

  if(microRtpsIsInit() == false)
  {
    if(microRtpsSetup() == false)
    {
      return NULL;
    }
  }

  for(idx = 0; idx < RTPS_SUBSCRIBER_MAX; idx++)
  {
    if(subscriber_table[idx].is_init == false)
    {
      p_sub = &subscriber_table[idx];
      break;
    }
  }
  if(idx == RTPS_SUBSCRIBER_MAX)
  {
    return NULL;
  }

  
  // Create Participant
  p_sub->participant_info = create_participant(rtps_client);
  send_to_agent(rtps_client);

  ret = microRtpsCheckRespose(p_sub->participant_info.object_id, CREATE_PARTICIPANT, 500);
  
  if(ret == true)
  {

    // Create Topic
    profile_xml.data = topic_profile;
    profile_xml.length = strlen(topic_profile);
    XRCEInfo topic_info = create_topic(rtps_client, p_sub->participant_info.object_id, profile_xml);
    send_to_agent(rtps_client);

    ret = microRtpsCheckRespose(topic_info.object_id, CREATE_TOPIC, timeout);

    if(ret == true)
    {
      // Create Subscriber
      p_sub->subscriber_info = create_subscriber(rtps_client, p_sub->participant_info.object_id);
      send_to_agent(rtps_client);

      ret = microRtpsCheckRespose(p_sub->subscriber_info.object_id, CREATE_SUBSCRIBER, timeout);

      if(ret == true)
      {
        // Create Reader
        profile_xml.data = data_reader_profile;
        profile_xml.length = strlen(data_reader_profile);
        p_sub->data_reader_info = create_data_reader(rtps_client, p_sub->participant_info.object_id, p_sub->subscriber_info.object_id, profile_xml);
        send_to_agent(rtps_client);

        ret = microRtpsCheckRespose(p_sub->data_reader_info.object_id, CREATE_READER, timeout);

        if(ret == true)
        {
          p_sub->func_deserialize = func_deserialize;
          p_sub->is_init = true;
        }
      }
    }
  }

  return p_sub->is_init == true ? p_sub : NULL;
}

void microRtpsWrite(RtpsPublisher_t* p_pub, void* topic)
{
  if(microRtpsIsInit() == false || p_pub == NULL)
  {
    return;
  }

  write_data(rtps_client, p_pub->data_writer_info.object_id, p_pub->func_serialize, topic);
  send_to_agent(rtps_client);
}

void microRtpsRead(RtpsSubscriber_t* p_sub, OnTopicReceived callback_func, void* callback_func_args)
{
  if(microRtpsIsInit() == false || p_sub == NULL)
  {
    return;
  }

  p_sub->read_callback_info = read_data(rtps_client, p_sub->data_reader_info.object_id, p_sub->func_deserialize, callback_func, callback_func_args);
  send_to_agent(rtps_client);
}

bool microRtpsCheckRespose(uint16_t object_id, RtpsProcess_t process, uint32_t timeout)
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
      case CREATE_TOPIC:
      case CREATE_PUBLISHER:
      case CREATE_SUBSCRIBER:
      case CREATE_WRITER:
      case CREATE_READER:
        if(ResponseTable.create.ids.object_id == object_id &&
            (ResponseTable.create.status == STATUS_OK || ResponseTable.create.status == STATUS_ERR_ALREADY_EXISTS))
        {
            ret = true;
        }
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





void microRtpslistenToAgent(void)
{
  if(rtps_client != NULL)
  {
    receive_from_agent(rtps_client);
  }
}