#include <ros2.hpp>

#include "std_msgs/String.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case STD_MSGS_STRING_TOPIC:
    {
      std_msgs::String topic;
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.print("Read topic: ");
      DEBUG_SERIAL.println(topic.data);
      break;
    }

    default:
      break;
  }
}


class StringPubSub : public ros2::Node
{
public:
  StringPubSub()
  : Node(), data_(false)
  {
    publisher_ = this->createPublisher<std_msgs::String>("String");
    subscriber_ = this->createSubscriber<std_msgs::String>("String");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    std_msgs::String string_topic;
    sprintf(string_topic.data, "String %d", (int)millis());
    publisher_->publish(&string_topic, STREAMID_BUILTIN_RELIABLE);
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<std_msgs::String>* publisher_;
  ros2::Subscriber<std_msgs::String>* subscriber_;
  bool data_;
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

    StringNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}