#include <ros2.hpp>

#include "HelloWorld.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


class HelloWorldPublisher : public ros2::Node
{
public:
  HelloWorldPublisher()
  : Node(), count_(0)
  {
    publisher_ = this->createPublisher<std_msg::HelloWorld>("HelloWorld");
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
  }

  ros2::Publisher<std_msg::HelloWorld>* publisher_;
  uint32_t count_;
};



void setup() 
{
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  while (!RTPS_SERIAL);

  DEBUG_SERIAL.println("Test Start");

  ros2::init();
}

void loop() 
{
  static uint32_t pre_time = 0;
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



