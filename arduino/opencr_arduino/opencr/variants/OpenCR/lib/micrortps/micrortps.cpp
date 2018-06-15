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


//-- Internal Variables
//

bool         g_is_rtps_init_done = false;
Session      g_rtps_session;
SessionId    g_session_id = 0x01;
ClientKey    g_client_key = {{0xAA, 0xAA, 0xAA, 0xAA}};



//-- Internal Functions
//


bool micrortps::setup(OnTopic callback)
{
  if(!new_serial_session(&g_rtps_session, g_session_id, g_client_key, "opencr_usb", callback, NULL))
  {
    return false;
  }

  g_is_rtps_init_done = init_session_sync(&g_rtps_session);

  return g_is_rtps_init_done;
}


bool micrortps::createParticipant(micrortps::Participant_t* participant, OnTopic callback)
{
  if(g_is_rtps_init_done == false)
  {
    if(micrortps::setup(callback) == false)
    {
      return NULL;
    }
  }

  participant->id.data[0] = 0x00;
  participant->id.data[1] = OBJK_PARTICIPANT;

  participant->is_init = create_participant_sync_by_ref(&g_rtps_session, participant->id, "default_participant", false, false);
  participant->session = &g_rtps_session;

  return participant->is_init;
}


bool micrortps::registerTopic(micrortps::Participant_t* participant, char* topic_profile)
{
  bool ret = false;
  ObjectId topic_id = {{0x00, OBJK_TOPIC}};

  ret = create_topic_sync_by_xml(&g_rtps_session, topic_id, topic_profile, participant->id, false, false);

  return ret;
}


bool micrortps::createPublisher(micrortps::Participant_t* participant, micrortps::Publisher_t* publisher, char* publisher_profile, char* writer_profile)
{
  bool ret;

  publisher->is_init = false;
  publisher->id.data[0] = 0x00;
  publisher->id.data[1] = OBJK_PUBLISHER;

  ret = create_publisher_sync_by_xml(&g_rtps_session, publisher->id, publisher_profile, participant->id, false, false);

  if (ret == true)
  {
    // Create Writer
    publisher->writer_id.data[0] = 0x00;
    publisher->writer_id.data[1] = OBJK_DATAWRITER;

    publisher->is_init = create_datawriter_sync_by_xml(&g_rtps_session, publisher->writer_id, writer_profile, publisher->id, false, false);
  }

  return publisher->is_init;
}


bool micrortps::createSubscriber(micrortps::Participant_t* participant, micrortps::Subscriber_t* subscriber, uint8_t topic_id, char* subscriber_profile, char* reader_profile)
{
  bool ret;

  subscriber->is_init = false;
  subscriber->id.data[0] = topic_id;
  subscriber->id.data[1] = OBJK_SUBSCRIBER;

  ret = create_subscriber_sync_by_xml(&g_rtps_session, subscriber->id, subscriber_profile, participant->id, false, false);

  if (ret == true)
  {
    // Create Writer
    subscriber->reader_id.data[0] = topic_id;
    subscriber->reader_id.data[1] = OBJK_DATAREADER;

    subscriber->is_init = create_datareader_sync_by_xml(&g_rtps_session, subscriber->reader_id, reader_profile, subscriber->id, false, false);
  }

  return subscriber->is_init;
}


void micrortps::subscribe(micrortps::Subscriber_t* subscriber, uint8_t StreamId)
{
  if(subscriber == NULL)
  {
    return;
  }
  read_data_sync(&g_rtps_session, subscriber->reader_id, StreamId);
}


void micrortps::runCommunication()
{
  run_communication(&g_rtps_session);
}
