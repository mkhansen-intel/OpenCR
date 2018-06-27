#include <ros2.hpp>

#include "geometry_msgs/Quaternion.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

static geometry_msgs::Quaternion topic;

void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case GEOMETRY_MSGS_QUATERNION_TOPIC:
    {
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.print(" Quaternion(x,y,z,w): ");
      DEBUG_SERIAL.print(topic.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.z); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.w);
      break;
    }

    default:
      break;
  }
}


class QuaternionPubSub : public ros2::Node
{
public:
  QuaternionPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<geometry_msgs::Quaternion>("Quaternion");
    subscriber_ = this->createSubscriber<geometry_msgs::Quaternion>("Quaternion");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 1;//micros()%128;
    quaternion.y = 2;//micros()%128;
    quaternion.z = 3;//micros()%128;
    quaternion.w = 4;//micros()%180;

    publisher_->publish(&quaternion, STREAMID_BUILTIN_RELIABLE);

    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<geometry_msgs::Quaternion>* publisher_;
  ros2::Subscriber<geometry_msgs::Quaternion>* subscriber_;
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
  static QuaternionPubSub QuaternionNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    QuaternionNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}