#include <ros2.hpp>

#include "test_msgs/Combination.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

static test_msgs::Combination topic;

void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case TEST_MSGS_COMBINATION_TOPIC:
    {
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.print(" Read topic: ");
      DEBUG_SERIAL.print(topic.message.data);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.println(topic.value.data);
      break;
    }

    default:
      break;
  }
}


class CombinationPubSub : public ros2::Node
{
public:
  CombinationPubSub()
  : Node(), data_(false), index_(0)
  {
    publisher_ = this->createPublisher<test_msgs::Combination>("Combination");
    subscriber_ = this->createSubscriber<test_msgs::Combination>("Combination");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    test_msgs::Combination combination_topic;
    data_ = !data_;
    combination_topic.value.data = data_;
    sprintf(message_, "String %d ", index_++);
    combination_topic.message.data = message_;

    publisher_->publish(&combination_topic, STREAMID_BUILTIN_RELIABLE);

    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<test_msgs::Combination>* publisher_;
  ros2::Subscriber<test_msgs::Combination>* subscriber_;
  bool data_;
  uint32_t index_;
  char message_[20];
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
  static CombinationPubSub CombinationNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    CombinationNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}