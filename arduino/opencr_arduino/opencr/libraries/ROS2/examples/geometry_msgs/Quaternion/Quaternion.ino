#include <ros2.hpp>

#include "geometry_msgs/Quaternion.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB



void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args);
static bool is_get_Quaternion_topic = false;



class QuaternionPubSub : public ros2::Node
{
public:
  QuaternionPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<geometry_msgs::Quaternion>("Quaternion");
    publisher_->setPublishInterval(2); // 2 hz
    subscriber_ = this->createSubscriber<geometry_msgs::Quaternion>("Quaternion");
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }


private:  
  void callback()
  {
    if(publisher_->isTimeToPublish())
    {
      callbackQuaternionPub();
    }

    if(is_get_Quaternion_topic)
    {
      subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
      is_get_Quaternion_topic = false;
    }
  }

  void callbackQuaternionPub(void)
  {
    geometry_msgs::Quaternion quaternion_topic;
    quaternion_topic.x = micros()%128;
    quaternion_topic.y = micros()%128;
    quaternion_topic.z = micros()%128;
    quaternion_topic.w = micros()%180;

    publisher_->publish(&quaternion_topic, STREAMID_BUILTIN_RELIABLE);
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

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin(&QuaternionNode);
}


void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case GEOMETRY_MSGS_QUATERNION_TOPIC:
    {
      geometry_msgs::Quaternion topic;

      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Quaternion(x,y,z,w): ");
      DEBUG_SERIAL.print(topic.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.z); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.w);

      is_get_Quaternion_topic = true;

      break;
    }

    default:
      break;
  }
}