#include <ros2.hpp>

#include "sensor_msgs/Imu.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

static sensor_msgs::Imu topic;

void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case SENSOR_MSGS_IMU_TOPIC:
    {
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
      break;
    }

    default:
      break;
  }
}


class ImuPubSub : public ros2::Node
{
public:
  ImuPubSub()
  : Node()
  {
    publisher_ = this->createPublisher<sensor_msgs::Imu>("Imu");
    subscriber_ = this->createSubscriber<sensor_msgs::Imu>("Imu");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    sensor_msgs::Imu imu_topic;
    imu_topic.header.frame_id = (char*) "OpenCR IMU";
    imu_topic.header.stamp.sec = millis()/1000;
    imu_topic.header.stamp.nanosec = (micros()%1000000)*1000;
    
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

    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<sensor_msgs::Imu>* publisher_;
  ros2::Subscriber<sensor_msgs::Imu>* subscriber_;
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

    ImuNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}