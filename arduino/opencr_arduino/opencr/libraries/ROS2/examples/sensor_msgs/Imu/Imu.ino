#include <ros2.hpp>

#include "sensor_msgs/Imu.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB



void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args);
static bool is_get_Imu_topic = false;


class ImuPubSub : public ros2::Node
{
public:
  ImuPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<sensor_msgs::Imu>("Imu");
    publisher_->setPublishInterval(2); // 2 hz
    subscriber_ = this->createSubscriber<sensor_msgs::Imu>("Imu");
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

private:  
  void callback()
  {
    if(publisher_->isTimeToPublish())
    {
      callbackImuPub();
    }

    if(is_get_Imu_topic)
    {
      subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
      is_get_Imu_topic = false;
    }

  }

  void callbackImuPub(void)
  {
    nano_time_ = get_nano_time();

    sensor_msgs::Imu imu_topic;
    imu_topic.header.frame_id = (char*) "OpenCR IMU";
    imu_topic.header.stamp.sec = nano_time_/1000000000;
    imu_topic.header.stamp.nanosec = nano_time_%1000000000;
    
    imu_topic.orientation.x = 1;
    imu_topic.orientation.y = 2;
    imu_topic.orientation.z = 3;
    imu_topic.orientation.w = 4;
    imu_topic.angular_velocity.x = 5;
    imu_topic.angular_velocity.y = 6;
    imu_topic.angular_velocity.z = 7;
    imu_topic.linear_acceleration.x = 8;
    imu_topic.linear_acceleration.y = 9;
    imu_topic.linear_acceleration.z = 10;

    publisher_->publish(&imu_topic, STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<sensor_msgs::Imu>* publisher_;
  ros2::Subscriber<sensor_msgs::Imu>* subscriber_;

  uint64_t nano_time_;
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
  static ImuPubSub ImuNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin(&ImuNode);
}



void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case SENSOR_MSGS_IMU_TOPIC:
    {
      sensor_msgs::Imu topic;

      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println(); 
      DEBUG_SERIAL.print(" Header(frameID,sec,nanosec): "); 
      DEBUG_SERIAL.print(topic.header.frame_id); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.header.stamp.sec); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.header.stamp.nanosec);
      DEBUG_SERIAL.print(" Orientation(x,y,z,w): ");
      DEBUG_SERIAL.print(topic.orientation.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.orientation.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.orientation.z); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.orientation.w); 
      DEBUG_SERIAL.print(" Angular velocity(x,y,z): ");
      DEBUG_SERIAL.print(topic.angular_velocity.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.angular_velocity.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.angular_velocity.z); 
      DEBUG_SERIAL.print(" Linear acceleration(x,y,z): ");
      DEBUG_SERIAL.print(topic.linear_acceleration.x); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.linear_acceleration.y); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.linear_acceleration.z); 

      is_get_Imu_topic = true;
      break;
    }

    default:
      break;
  }
}