#include <ros2.hpp>

#include "HelloWorld.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case HELLO_WORLD_TOPIC:
    {
      std_msg::HelloWorld topic;
      deserialize_HelloWorld_topic(serialized_topic, &topic.message_);
      DEBUG_SERIAL.print("Read topic: ");
      DEBUG_SERIAL.print(topic.message_.message);
      DEBUG_SERIAL.print(", count: ");
      DEBUG_SERIAL.println(topic.message_.index);
      break;
    }

    default:
      break;
  }
}


class HelloWorldPublisher : public ros2::Node
{
public:
  HelloWorldPublisher()
  : Node(), count_(0)
  {
    publisher_ = this->createPublisher<std_msg::HelloWorld>("HelloWorld");
    subscriber_ = this->createSubscriber<std_msg::HelloWorld>("HelloWorld");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    std_msg::HelloWorld hello_topic;
    hello_topic.message_.message = (char*) "HelloWorld";
    hello_topic.message_.index = count_++;
    publisher_->publish(&hello_topic, STREAMID_BUILTIN_RELIABLE);
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<std_msg::HelloWorld>* publisher_;
  ros2::Subscriber<std_msg::HelloWorld>* subscriber_;
  uint32_t count_;
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
  static HelloWorldPublisher HelloWorldNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    HelloWorldNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}
