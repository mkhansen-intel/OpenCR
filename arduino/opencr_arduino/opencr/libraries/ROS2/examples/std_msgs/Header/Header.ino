#include <ros2.hpp>

#include "std_msgs/Header.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case STD_MSGS_HEADER_TOPIC:
    {
      std_msgs::Header topic;
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Header(frame_id,sec,nanosec): ");
      DEBUG_SERIAL.print(topic.frame_id); DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(topic.stamp.sec); DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.println(topic.stamp.nanosec); 
      break;
    }

    default:
      break;
  }
}


class HeaderPubSub : public ros2::Node
{
public:
  HeaderPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<std_msgs::Header>("Header");
    subscriber_ = this->createSubscriber<std_msgs::Header>("Header");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    std_msgs::Header header_topic;
    header_topic.stamp.sec = millis()/1000;
    header_topic.stamp.nanosec = (micros()%1000000)*1000;
    header_topic.frame_id = (char*) "OpenCR Frame ID";

    publisher_->publish(&header_topic, STREAMID_BUILTIN_RELIABLE);
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<std_msgs::Header>* publisher_;
  ros2::Subscriber<std_msgs::Header>* subscriber_;
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
  static HeaderPubSub HeaderNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    HeaderNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}