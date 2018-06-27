#include <ros2.hpp>

#include "sensor_msgs/LaserScan.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB

static sensor_msgs::LaserScan topic;

void on_topic(ObjectId id, MicroBuffer* serialized_topic, void* args)
{
  ((void)(args));

  switch(id.data[0])
  {
    case SENSOR_MSGS_LASER_SCAN_TOPIC:
    {
      topic.deserialize(serialized_topic, &topic);
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Header(frameID,sec,nanosec): "); 
      DEBUG_SERIAL.print(topic.header.frame_id); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.header.stamp.sec); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.header.stamp.nanosec);
      DEBUG_SERIAL.print(" Angle(min,max,inc): ");
      DEBUG_SERIAL.print(topic.angle_min); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic.angle_max); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.angle_increment);
      DEBUG_SERIAL.print(" Time(inc,scan): ");
      DEBUG_SERIAL.print(topic.time_increment); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.scan_time);
      DEBUG_SERIAL.print(" Range(min,max): ");
      DEBUG_SERIAL.print(topic.range_min); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic.range_max);
      DEBUG_SERIAL.print(" Range(data:size): ");
      for(uint32_t i = 0; i < topic.ranges_size; i++)
      {
        DEBUG_SERIAL.print(topic.ranges_data[i]); DEBUG_SERIAL.print(" ");
      }
      DEBUG_SERIAL.print(": ");
      DEBUG_SERIAL.println(topic.ranges_size);
      DEBUG_SERIAL.print(" Intensity(data:size): ");
      for(uint32_t i = 0; i < topic.intensities_size; i++)
      {
        DEBUG_SERIAL.print(topic.intensities_data[i]); DEBUG_SERIAL.print(" ");
      }
      DEBUG_SERIAL.print(": ");
      DEBUG_SERIAL.println(topic.intensities_size);
      break;
    }

    default:
      break;
  }
}


class LaserScanPubSub : public ros2::Node
{
public:
  LaserScanPubSub()
  : Node()
  {
    memset(ranges_data_, 0, sizeof(ranges_data_));
    memset(intensities_data, 0, sizeof(intensities_data));

    publisher_ = this->createPublisher<sensor_msgs::LaserScan>("LaserScan");
    subscriber_ = this->createSubscriber<sensor_msgs::LaserScan>("LaserScan");
  }

  void run(void)
  {
    this->timer_callback();
  }

private:  
  void timer_callback()
  {
    sensor_msgs::LaserScan laser_scan_topic;
    laser_scan_topic.header.frame_id = (char*) "OpenCR LaserScan";
    laser_scan_topic.header.stamp.sec = millis()/1000;
    laser_scan_topic.header.stamp.nanosec = (micros()%1000000)*1000;
    
    laser_scan_topic.angle_min = 1;
    laser_scan_topic.angle_max = 2;
    laser_scan_topic.angle_increment = 3;
    laser_scan_topic.time_increment = 4;
    laser_scan_topic.scan_time = 5;
    laser_scan_topic.range_min = 6;
    laser_scan_topic.range_max = 7;

    laser_scan_topic.ranges_data = ranges_data_;
    laser_scan_topic.ranges_size = sizeof(ranges_data_)/sizeof(float);
    for(uint32_t i = 0; i < laser_scan_topic.ranges_size; i++)
    {
      laser_scan_topic.ranges_data[i] = (float)(micros()%128);
    }

    laser_scan_topic.intensities_data = intensities_data;
    laser_scan_topic.intensities_size = sizeof(intensities_data)/sizeof(float);
    for(uint32_t i = 0; i < laser_scan_topic.intensities_size; i++)
    {
      laser_scan_topic.intensities_data[i] = (float)(micros()%128);
    }

    publisher_->publish(&laser_scan_topic, STREAMID_BUILTIN_RELIABLE);

    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }

  ros2::Publisher<sensor_msgs::LaserScan>* publisher_;
  ros2::Subscriber<sensor_msgs::LaserScan>* subscriber_;

  float ranges_data_[8];
  float intensities_data[8];
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
  static LaserScanPubSub LaserScanNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    LaserScanNode.run();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin();
}