#include <ros2.hpp>

#include "std_msgs/String.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args);
static bool is_get_String_topic = false;




class StringPubSub : public ros2::Node
{
public:
  StringPubSub()
  : Node(), cnt_(0)
  {
    publisher_ = this->createPublisher<std_msgs::String>("String");
    publisher_->setPublishInterval(2); // 2 hz
    subscriber_ = this->createSubscriber<std_msgs::String>("String");
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

private:  
  void callback()
  {
    if(publisher_->isTimeToPublish())
    {
      callbackStringPub();
    }

    if(is_get_String_topic)
    {
      subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
      is_get_String_topic = false;
    }
  }

  void callbackStringPub(void)
  {
    std_msgs::String string_topic;
    sprintf(string_topic.data, "String %d", cnt_++);
    publisher_->publish(&string_topic, STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<std_msgs::String>* publisher_;
  ros2::Subscriber<std_msgs::String>* subscriber_;
  int cnt_;
};



void setup() 
{
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  while (!RTPS_SERIAL);

  ros2::init(on_topic);
}

void loop() 
{
  static uint32_t pre_time = millis();
  static bool led_state = false;
  static StringPubSub StringNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin(&StringNode);
}


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case STD_MSGS_STRING_TOPIC:
    {
      std_msgs::String topic;

      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" String: ");
      DEBUG_SERIAL.println(topic.data);
      
      is_get_String_topic = true;
      
      break;
    }

    default:
      break;
  }
}
