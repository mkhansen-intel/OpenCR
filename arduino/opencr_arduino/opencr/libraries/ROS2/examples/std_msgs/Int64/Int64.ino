#include <ros2.hpp>

#include "std_msgs/Int64.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB



class Int64PubSub : public ros2::Node
{
public:
  Int64PubSub()
  : Node()
  {
    publisher_ = this->createPublisher<std_msgs::Int64>("Int64");
    publisher_->setPublishInterval(2); // 2 hz
    subscriber_ = this->createSubscriber<std_msgs::Int64>("Int64");
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

private:  
  void timerCallback()
  {
    if(publisher_->isTimeToPublish())
    {
      std_msgs::Int64 int64_topic;
      int64_topic.data = (int64_t)get_nano_time();

      publisher_->publish(&int64_topic, STREAMID_BUILTIN_RELIABLE);
    }
  }

  void userTopicCallback(uint8_t topic_id, void* topic_msg)
  {

    if(topic_id == subscriber_->topic_id_)
    {
      std_msgs::Int64 *topic = (std_msgs::Int64*)topic_msg;

      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Int64: ");
      DEBUG_SERIAL.print((int32_t)(topic->data >> 32));
      DEBUG_SERIAL.println((int32_t)(topic->data));

      subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
    }
  }

  ros2::Publisher<std_msgs::Int64>* publisher_;
  ros2::Subscriber<std_msgs::Int64>* subscriber_;
};



void setup() 
{
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  while (!RTPS_SERIAL);

  ros2::init();
}

void loop() 
{
  static uint32_t pre_time = millis();
  static bool led_state = false;
  static Int64PubSub Int64Node;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin(&Int64Node);
}