/*
 *  micrortps.cpp
 *
 *  Created on: 2018. 5. 25.
 *      Author: Kei
 */

#include <stdlib.h>
#include <string.h>

#include "micrortps.hpp"
#include "hw.h"


const uint32_t RTPS_BUFFER_SIZE      = 4096;
const uint32_t RTPS_NODE_MAX         = 5;
const uint32_t RTPS_RESPONSE_WAIT_MS = 500; 

//-- Internal Variables
//
namespace micrortps {

typedef struct
{
  XRCEInfo ids;
  uint8_t  operation;
  uint8_t  status;
} Response_t;

typedef struct
{
  Response_t create;
  Response_t update;
  Response_t deleted;
  Response_t lookup;
  Response_t read;
  Response_t write;
} ResponseTable_t;

typedef enum
{
  CREATE_CLIENT = 0,
  CREATE_PARTICIPANT,
  CREATE_TOPIC,
  CREATE_PUBLISHER,
  CREATE_SUBSCRIBER,
  CREATE_WRITER,
  CREATE_READER,
} Process_t;

} // namespace micrortps

bool is_rtps_init_done = false;
ClientState    *rtps_client;
XRCEInfo        client_info;
micrortps::ResponseTable_t ResponseTable;


//-- Internal Functions
//
static bool checkCreatedResponse(XRCEInfo info, micrortps::Process_t process, uint32_t timeout);
static void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args);


bool micrortps::setup(void)
{
  if(rtps_client == NULL)
  {
    rtps_client = new_serial_client_state(RTPS_BUFFER_SIZE, "opencr_usb");
  }

  client_info = create_client(rtps_client, on_status_received, &ResponseTable);
  send_to_agent(rtps_client);

  is_rtps_init_done = checkCreatedResponse(client_info, CREATE_CLIENT, RTPS_RESPONSE_WAIT_MS);

  return is_rtps_init_done;
}


bool micrortps::createParticipant(Participant_t* participant)
{
  if(is_rtps_init_done == false)
  {
    if(micrortps::setup() == false)
    {
      return NULL;
    }
  }

  participant->info = create_participant(rtps_client);
  send_to_agent(rtps_client);

  participant->is_init = checkCreatedResponse(participant->info, CREATE_PARTICIPANT, RTPS_RESPONSE_WAIT_MS);

  return participant->is_init;
}


bool micrortps::registerTopic(Participant_t* participant, char* topic_profile)
{
  bool ret = false;
  string profile_xml;
  XRCEInfo topic_info;

  profile_xml.data = topic_profile;
  profile_xml.length = strlen(topic_profile);
  topic_info = create_topic(rtps_client, participant->info.object_id, profile_xml);
  send_to_agent(rtps_client);

  ret = checkCreatedResponse(topic_info, CREATE_TOPIC, RTPS_RESPONSE_WAIT_MS);

  return ret;
}


bool micrortps::createPublisher(Participant_t* participant, Publisher_t* publisher, char* writer_profile)
{
  bool ret;
  string profile_xml;

  publisher->is_init = false;
  publisher->info = create_publisher(rtps_client, participant->info.object_id);
  send_to_agent(rtps_client);

  ret = checkCreatedResponse(publisher->info, micrortps::CREATE_PUBLISHER,
        RTPS_RESPONSE_WAIT_MS);

  if (ret == true)
  {
    // Create Writer
    profile_xml.data = writer_profile;
    profile_xml.length = strlen(writer_profile);
    publisher->writer_info = create_data_writer(rtps_client,
        publisher->info.object_id, publisher->info.object_id, profile_xml);

    send_to_agent(rtps_client);
    ret = checkCreatedResponse(publisher->writer_info, micrortps::CREATE_WRITER,
        RTPS_RESPONSE_WAIT_MS);

    publisher->is_init = ret;
  }

  return publisher->is_init;
}


bool micrortps::createSubscriber(Participant_t* participant, Subscriber_t* subscriber, char* reader_profile)
{
  bool ret;
  string profile_xml;

  subscriber->is_init = false;
  subscriber->info = create_subscriber(rtps_client, participant->info.object_id);
  send_to_agent(rtps_client);

  ret = checkCreatedResponse(subscriber->info, micrortps::CREATE_SUBSCRIBER,
        RTPS_RESPONSE_WAIT_MS);

  if (ret == true)
  {
    // Create Reader
    profile_xml.data = reader_profile;
    profile_xml.length = strlen(reader_profile);
    subscriber->reader_info = create_data_reader(rtps_client,
        participant->info.object_id, subscriber->info.object_id, profile_xml);
  
    send_to_agent(rtps_client);
    ret = checkCreatedResponse(subscriber->reader_info, micrortps::CREATE_READER,
        RTPS_RESPONSE_WAIT_MS);

    subscriber->is_init = ret;
  }

  return subscriber->is_init;
}


void micrortps::publish(Publisher_t* publisher, SerializeTopic func_serialize, void* topic)
{
  if(publisher == NULL)
  {
    return;
  }

  write_data(rtps_client, publisher->writer_info.object_id, func_serialize, topic);
  send_to_agent(rtps_client);
}


void micrortps::subscribe(Subscriber_t* subscriber, DeserializeTopic func_deserialize, OnTopicReceived callback_func, void* callback_func_args)
{
  if(subscriber == NULL)
  {
    return;
  }

  subscriber->read_callback_info = 
    read_data(rtps_client, subscriber->reader_info.object_id, func_deserialize, callback_func, callback_func_args);
  send_to_agent(rtps_client);
}


void micrortps::listenFromAgent(void)
{
  if(rtps_client != NULL)
  {
    receive_from_agent(rtps_client);
  }
}



static bool checkCreatedResponse(XRCEInfo info, micrortps::Process_t process, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time;

  pre_time = millis();
  while((millis() - pre_time < timeout))
  {
    receive_from_agent(rtps_client);

    switch(process)
    {
      case micrortps::CREATE_CLIENT :
      case micrortps::CREATE_PARTICIPANT:
      case micrortps::CREATE_PUBLISHER:
      case micrortps::CREATE_SUBSCRIBER:
      case micrortps::CREATE_WRITER:
      case micrortps::CREATE_READER:
        if(ResponseTable.create.ids.object_id == info.object_id && ResponseTable.create.status == STATUS_OK)
        {
          ret = true;
        }
        break;

      case micrortps::CREATE_TOPIC:
        if(ResponseTable.create.ids.object_id == info.object_id)
        {
          ret = true;
        }
        break;

      default:
        break;
    }

    if(ret == true)
    {
      break;
    }
  }
  return ret;
}


static void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args)
{
  micrortps::ResponseTable_t *tbl = (micrortps::ResponseTable_t *) args;
  micrortps::Response_t *recv;

  if(tbl == NULL)
  {
    return;
  }

  switch(operation)
  {
    case STATUS_LAST_OP_CREATE:
      recv = (micrortps::Response_t *) &tbl->create;
      break;
    case STATUS_LAST_OP_UPDATE:
      recv = (micrortps::Response_t *) &tbl->update;
      break;
    case STATUS_LAST_OP_DELETE:
      recv = (micrortps::Response_t *) &tbl->deleted;
      break;
    case STATUS_LAST_OP_LOOKUP:
      recv = (micrortps::Response_t *) &tbl->lookup;
      break;
    case STATUS_LAST_OP_READ:
      recv = (micrortps::Response_t *) &tbl->read;
      break;
    case STATUS_LAST_OP_WRITE:
      recv = (micrortps::Response_t *) &tbl->write;
      break;
    default :
      return;
  }

  recv->ids = info;
  recv->operation = operation;
  recv->status = status;
}


