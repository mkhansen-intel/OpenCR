#include <ros2.hpp>

#include "test_msgs/Complex.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case TEST_MSGS_COMPLEX_TOPIC:
    {
      test_msgs::Complex topic;
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.print("Read topic: ");
      DEBUG_SERIAL.print(topic.message.data);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.println(topic.value.data);
      break;
    }

    default:
      break;
  }
}


class ComplexPubSub : public ros2::Node
{
public:
  ComplexPubSub()
  : Node(), data_(false)
  {
    publisher_ = this->createPublisher<test_msgs::Complex>("Complex");
    subscriber_ = this->createSubscriber<test_msgs::Complex>("Complex");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    test_msgs::Complex complex_topic;
    data_ = !data_;
    complex_topic.value.data = data_;
    sprintf(complex_topic.message.data, "String %d", (int)millis());
    publisher_->publish(&complex_topic, STREAMID_BUILTIN_RELIABLE);
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<test_msgs::Complex>* publisher_;
  ros2::Subscriber<test_msgs::Complex>* subscriber_;
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
  static ComplexPubSub ComplexNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    ComplexNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}