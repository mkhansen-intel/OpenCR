#include <ros2.hpp>

#include "std_msgs/Bool.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case STD_MSGS_BOOL_TOPIC:
    {
      std_msgs::Bool topic;
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Bool: ");
      DEBUG_SERIAL.println(topic.data);
      break;
    }

    default:
      break;
  }
}


class BoolPubSub : public ros2::Node
{
public:
  BoolPubSub()
  : Node(), data_(false)
  {
    publisher_ = this->createPublisher<std_msgs::Bool>("Bool");
    subscriber_ = this->createSubscriber<std_msgs::Bool>("Bool");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    std_msgs::Bool bool_topic;
    data_ = !data_;
    bool_topic.data = data_;
    publisher_->publish(&bool_topic, STREAMID_BUILTIN_RELIABLE);
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<std_msgs::Bool>* publisher_;
  ros2::Subscriber<std_msgs::Bool>* subscriber_;
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
  static BoolPubSub BoolNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    BoolNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}