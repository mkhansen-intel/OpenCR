#include <ros2.hpp>

#include "sensor_msgs/BatteryState.hpp"


#define DEBUG_SERIAL Serial2  
#define RTPS_SERIAL  Serial   //OpenCR USB



class BatteryStatePubSub : public ros2::Node
{
public:
  BatteryStatePubSub()
  : Node()
  {
    memset(cell_voltage_data_, 0, sizeof(cell_voltage_data_));

    publisher_ = this->createPublisher<sensor_msgs::BatteryState>("BatteryState");
    publisher_->setPublishInterval(2); // 2 hz
    subscriber_ = this->createSubscriber<sensor_msgs::BatteryState>("BatteryState");
    subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
  }


private:

  void timerCallback()
  {
    if(publisher_->isTimeToPublish())
    {
      nano_time_ = ros2::getNanoTime();

      sensor_msgs::BatteryState battery_state_topic;
      battery_state_topic.header.frame_id = (char*) "OpenCR BatteryState";
      battery_state_topic.header.stamp.sec = (int32_t)(nano_time_/(uint64_t)1000000000UL);
      battery_state_topic.header.stamp.nanosec = (uint32_t)(nano_time_%(uint64_t)1000000000UL);

      battery_state_topic.voltage = 1;
      battery_state_topic.current = 2;
      battery_state_topic.charge = 3;
      battery_state_topic.capacity = 4;
      battery_state_topic.design_capacity = 5;
      battery_state_topic.percentage = 6;

      battery_state_topic.power_supply_status = 0;
      battery_state_topic.power_supply_health = 1;
      battery_state_topic.power_supply_technology = 2;

      battery_state_topic.present = true;
      battery_state_topic.cell_voltage = cell_voltage_data_;
      battery_state_topic.cell_voltage_size = sizeof(cell_voltage_data_)/sizeof(float);
      for(uint32_t i = 0; i < battery_state_topic.cell_voltage_size; i++)
      {
        battery_state_topic.cell_voltage[i] = 3.7;
      }

      battery_state_topic.location = (char*)"Seoul";
      battery_state_topic.serial_number = (char*)"123-456-789";

      if(publisher_ != NULL)
      {
        publisher_->publish(&battery_state_topic, STREAMID_BUILTIN_RELIABLE);
      }
    }
  }

  void userTopicCallback(uint8_t topic_id, void* topic_msg)
  {
    if(topic_id == subscriber_->topic_id_)
    {
      sensor_msgs::BatteryState* topic = (sensor_msgs::BatteryState*)topic_msg;

      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(" Header(frameID,sec,nanosec): "); 
      DEBUG_SERIAL.print(topic->header.frame_id); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->header.stamp.sec); DEBUG_SERIAL.print("."); 
      DEBUG_SERIAL.println(topic->header.stamp.nanosec);
      DEBUG_SERIAL.print(" Value(voltage,current,charge,capacity,design capacity,percentage): ");
      DEBUG_SERIAL.print(topic->voltage); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->current); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->charge); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->capacity); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->design_capacity); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic->percentage);
      DEBUG_SERIAL.print(" Info(status,health,technology,present): ");
      DEBUG_SERIAL.print(topic->power_supply_status); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->power_supply_health); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.print(topic->power_supply_technology); DEBUG_SERIAL.print(","); 
      DEBUG_SERIAL.println(topic->present);
      DEBUG_SERIAL.print(" CellInfo(cell voltage:size): ");
      for(uint32_t i = 0; i < topic->cell_voltage_size; i++)
      {
        DEBUG_SERIAL.print(topic->cell_voltage[i]); DEBUG_SERIAL.print(" "); 
      }
      DEBUG_SERIAL.print(": "); 
      DEBUG_SERIAL.println(topic->cell_voltage_size);
      DEBUG_SERIAL.print(" Location: ");
      DEBUG_SERIAL.println(topic->location);
      DEBUG_SERIAL.print(" Serial Number ");
      DEBUG_SERIAL.println(topic->serial_number);

      subscriber_->subscribe(STREAMID_BUILTIN_RELIABLE);
    }
  }


  ros2::Publisher<sensor_msgs::BatteryState>* publisher_;
  ros2::Subscriber<sensor_msgs::BatteryState>* subscriber_;

  float cell_voltage_data_[3];
  uint64_t nano_time_;
};


void setup() 
{
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  while (!RTPS_SERIAL);

  ros2::init();
}

void loop() 
{
  static uint32_t pre_time = millis();
  static bool led_state = false;
  static BatteryStatePubSub BatteryStateNode;

  if(millis() - pre_time > 500)
  {
    pre_time = millis();

    digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
  }

  ros2::spin(&BatteryStateNode);
}