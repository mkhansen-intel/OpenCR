#include "micrortps.h"

#include "HelloWorld.h"
#include "xml.h"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

void on_hello_topic(XRCEInfo info, const void* vtopic, void* args);


RtpsPublisher_t *pub;
RtpsSubscriber_t *sub;

void setup() 
{
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  while (!RTPS_SERIAL);

  microRtpsSetup();
  pub = microRtpsCreatePub(topic_xml, writer_xml, serialize_HelloWorld_topic, 500);
  sub = microRtpsCreateSub(topic_xml, reader_xml, deserialize_HelloWorld_topic, 500);
}


uint32_t pre_time = 0;
bool led_state = false;
HelloWorld hello_topic = {0, (char *) "HelloWorld"};

void loop() 
{
  if(millis() - pre_time > 500)
  {
    pre_time = millis();
    hello_topic.m_index++;
    microRtpsWrite(pub, &hello_topic);
    microRtpsRead(sub, on_hello_topic, NULL);
  }

  microRtpslistenToAgent();

  // Check whether init or not init.
  if(pub == NULL)
  {
    pub = microRtpsCreatePub(topic_xml, writer_xml, serialize_HelloWorld_topic, 500);
  }
  if(sub == NULL)
  {
    sub = microRtpsCreateSub(topic_xml, reader_xml, deserialize_HelloWorld_topic, 500);
  }
}


void on_hello_topic(XRCEInfo info, const void* vtopic, void* args)
{
  UNUSED(info);

  HelloWorld* topic = (HelloWorld*) vtopic;

  if(args != NULL)
  {
    HelloWorld* recv = (HelloWorld*) args;

    recv->m_index = topic->m_index;
    recv->m_message = topic->m_message;
  }

  DEBUG_SERIAL.print("- Message ");
  DEBUG_SERIAL.print(topic->m_message);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(topic->m_index);
  DEBUG_SERIAL.println(" RECEIVED");

  free(topic->m_message);
  free(topic);
}

