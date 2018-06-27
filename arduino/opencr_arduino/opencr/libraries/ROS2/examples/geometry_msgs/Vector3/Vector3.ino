#include <ros2.hpp>

#include "geometry_msgs/Vector3.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

static geometry_msgs::Vector3 topic;

void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case GEOMETRY_MSGS_VECTOR3_TOPIC:
    {
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Vector3(x,y,z): ");
      DEBUG_SERIAL.print(topic.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.z);
      break;
    }

    default:
      break;
  }
}


class Vector3PubSub : public ros2::Node
{
public:
  Vector3PubSub()
  : Node()
  {
    publisher_ = this->createPublisher<geometry_msgs::Vector3>("Vector3");
    subscriber_ = this->createSubscriber<geometry_msgs::Vector3>("Vector3");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    geometry_msgs::Vector3 vector3_topic;
    vector3_topic.x = millis()%128;
    vector3_topic.y = millis()%128;
    vector3_topic.z = millis()%128;

    publisher_->publish(&vector3_topic, STREAMID_BUILTIN_RELIABLE);

    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<geometry_msgs::Vector3>* publisher_;
  ros2::Subscriber<geometry_msgs::Vector3>* subscriber_;
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
  static Vector3PubSub Vector3Node;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    Vector3Node.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}