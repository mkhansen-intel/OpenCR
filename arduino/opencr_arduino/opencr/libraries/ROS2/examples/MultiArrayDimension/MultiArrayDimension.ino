#include <ros2.hpp>

#include "std_msgs/MultiArrayDimension.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case STD_MSGS_MULTI_ARRAY_DIMENSION_TOPIC:
    {
      std_msgs::MultiArrayDimension topic;
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.print("Read topic: ");
      DEBUG_SERIAL.print(topic.label);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(topic.size);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.println(topic.stride);
      break;
    }

    default:
      break;
  }
}


class MultiArrayDimensionPubSub : public ros2::Node
{
public:
  MultiArrayDimensionPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<std_msgs::MultiArrayDimension>("rt/MultiArrayDimensionPub");
    subscriber_ = this->createSubscriber<std_msgs::MultiArrayDimension>("rt/MultiArrayDimensionSub");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    std_msgs::MultiArrayDimension multi_array_dimension_topic;

    multi_array_dimension_topic.label = (char*) "MultiArrayDimension";
    multi_array_dimension_topic.size = strlen(multi_array_dimension_topic.label);
    multi_array_dimension_topic.stride = millis(); //for test

    publisher_->publish(&multi_array_dimension_topic, STREAMID_BUILTIN_RELIABLE);
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<std_msgs::MultiArrayDimension>* publisher_;
  ros2::Subscriber<std_msgs::MultiArrayDimension>* subscriber_;
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
  static MultiArrayDimensionPubSub MultiArrayDimensionNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    MultiArrayDimensionNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}