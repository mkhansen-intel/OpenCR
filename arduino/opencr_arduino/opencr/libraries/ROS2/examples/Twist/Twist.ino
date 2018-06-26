#include <ros2.hpp>

#include "geometry_msgs/Twist.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

static geometry_msgs::Twist topic;

void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case GEOMETRY_MSGS_TWIST_TOPIC:
    {
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.print(" Linear(x,y,z): ");
      DEBUG_SERIAL.print(topic.linear.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.linear.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.linear.z); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(", Angular(x,y,z)");
      DEBUG_SERIAL.print(topic.angular.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.angular.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.angular.z); DEBUG_SERIAL.print(","); 
      break;
    }

    default:
      break;
  }
}


class TwistPubSub : public ros2::Node
{
public:
  TwistPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<geometry_msgs::Twist>("rt/TwistPub");
    subscriber_ = this->createSubscriber<geometry_msgs::Twist>("rt/TwistSub");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    geometry_msgs::Twist twist_topic;
    twist_topic.linear.x = micros()%128;
    twist_topic.linear.y = micros()%128;
    twist_topic.linear.z = micros()%128;
    twist_topic.angular.x = micros()%180;
    twist_topic.angular.y = micros()%180;
    twist_topic.angular.z = micros()%180;

    publisher_->publish(&twist_topic, STREAMID_BUILTIN_RELIABLE);

    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<geometry_msgs::Twist>* publisher_;
  ros2::Subscriber<geometry_msgs::Twist>* subscriber_;
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
  static TwistPubSub TwistNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    TwistNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}