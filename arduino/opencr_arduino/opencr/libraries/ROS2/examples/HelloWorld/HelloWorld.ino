#include "micrortps.h"

#include "HelloWorld.h"
#include "xml.h"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

void on_hello_topic(XRCEInfo info, const void* vtopic, void* args);


RtpsNode_t *node = NULL;
RtpsPublisher_t *pub = NULL;
RtpsSubscriber_t *sub = NULL;

void setup() 
{
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  while (!RTPS_SERIAL);

  DEBUG_SERIAL.println("Test Start");

  uRtpsSetup();
  node = uRtpsCreateNode();
  pub = uRtpsCreatePub(node, HelloWorldGetInfo(), writer_xml, 500);
  sub = uRtpsCreateSub(node, HelloWorldGetInfo(), reader_xml, 500);
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
    uRtpsWrite(pub, &hello_topic);
    uRtpsRead(sub, on_hello_topic, NULL);
    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  uRtpslistenToAgent();

  // Check whether init or not init.
  if(pub == NULL)
  {
    pub = uRtpsCreatePub(node, HelloWorldGetInfo(), writer_xml);
  }
  if(sub == NULL)
  {
    sub = uRtpsCreateSub(node, HelloWorldGetInfo(), reader_xml);
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
