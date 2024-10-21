#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#include <Ticker.h>
#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/laser_scan.h>

int LED_BUILTIN = 2;
byte Sync0[1];
byte Sync1[1];
byte Sync[2];
byte lidarData[2520];
byte remainingData[2518];
int received_length = 0;
byte chksm = 0;
int good_packet = 0;
int old_good_packet = 0;
int rpm = 0;

#define DO_CHKSM 1

HardwareSerial & lidar = Serial2;

// Create accelerometer and magnetometer identifiers
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;
Adafruit_10DOF dof = Adafruit_10DOF();

Ticker accessOrientation;

// microROS
rcl_publisher_t publisher;
rcl_publisher_t laserPublisher;
std_msgs__msg__Int32 msg;
sensor_msgs__msg__LaserScan laserScan;

rclc_executor_t executor;
//rclc_executor_t laserExecutor;
rclc_support_t support;
//rclc_support_t laserSupport;
rcl_allocator_t allocator;
//rcl_allocator_t laserAllocator;
rcl_node_t node;
rcl_timer_t timer;



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

// function to get the orientation of the IMU
void getOrientation(){
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  if(dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)){
    //Serial.println("Got orientation");  
  };
  

}

void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lidar.begin(230400);
    

  // Begin connections to accelerometer & magnetometer and attach 50ms interrupt to callback
  accel.begin();
  mag.begin();
  accessOrientation.attach_ms(50, getOrientation);

  // Configure the microROS transportation type
  set_microros_serial_transports(Serial);
  delay(2000);

  const int timeout_ms = 2000;    // timeout for pinging Agent & syncing time
  const uint8_t attempts = 5;    // Number of attempts

  // Ping the agent
  rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

  // Keep pinging until connected
  while(ping_result == RMW_RET_ERROR) {
    ping_result = rmw_uros_ping_agent(timeout_ms, attempts);
  }


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &laserPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "laser_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_init(&laserExecutor, &laserSupport.context, 1, &laserAllocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;

   // Synchronize time with the agent
  rmw_uros_sync_session(timeout_ms);
  if(!rmw_uros_epoch_synchronized()){
    error_loop();
  }
  
  int64_t time_ns = rmw_uros_epoch_nanos();
  int64_t time_ms = rmw_uros_epoch_millis();

  // Assigning dynamic memory to the frame_id char sequence
  laserScan.header.frame_id.capacity = 20;
  laserScan.header.frame_id.data = (char*) malloc(laserScan.header.frame_id.capacity * sizeof(char));
  laserScan.header.frame_id.size = 0;
  
  // Fill laserScan message with initial data
  strcpy(laserScan.header.frame_id.data, "laser_frame");
  laserScan.header.frame_id.size = strlen(laserScan.header.frame_id.data);
  laserScan.header.stamp.nanosec = time_ns;
  laserScan.header.stamp.sec = time_ms / 1000.0;
  laserScan.angle_min = 0;
  laserScan.angle_max = 2*PI;   // in rads
  laserScan.angle_increment = 1.0 * (PI/180.0); // in rads
  laserScan.time_increment = 1.0 / (5.0 * 360.0); // time between measurements (1 second divided by revs per second diveded by number of measurements each scan)
  laserScan.scan_time = 1.0 / 5.0;    // time between scans (should be adjusted with speed changes) [1 second divided by revs per second]
  laserScan.range_min = 0.12;   // in metres
  laserScan.range_max = 3.50;   // in metres
  laserScan.ranges.capacity = 360;
  laserScan.ranges.data = (float*) malloc(laserScan.ranges.capacity * sizeof(float));
  laserScan.ranges.size = 0;

  lidar.write('b');

}


void loop() {


  digitalWrite(LED_BUILTIN, HIGH);

  int64_t time_ns = rmw_uros_epoch_nanos();
  int64_t time_ms = rmw_uros_epoch_millis();
  laserScan.header.stamp.nanosec = time_ns;
  laserScan.header.stamp.sec = time_ms / 1000;

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // keep reading data until find very first packets 0xFA, 0xA0
  while((Sync0[0] != 0xFA) || (Sync1[0] != 0xA0)) {
    
    // keep reading single byte until finding sync byte "0xFA"
    received_length = lidar.readBytes(Sync0, 1);
    
    // read second byte if first sync byte found
    if(Sync0[0] == 0xFA){
      received_length = lidar.readBytes(Sync1, 1);
    }

  }
  

  // now we have first two sync byte of lidar, read remaining data.
  // 2520 total bytes (42 bytes and 60 packets) (2518 + 2 sync)
  if(Sync0[0] == 0xFA && Sync1[0] == 0xA0){
    received_length = lidar.readBytes(remainingData, 2518);
  }

  Sync[0] = Sync0[0];
  Sync[1] = Sync1[0];

  // create new array containing full 2520 packets called lidarData, combing Sync & remainingData
  memcpy(lidarData, Sync, sizeof(Sync));
  memcpy(lidarData+sizeof(Sync), remainingData, sizeof(remainingData));

  // check sum. Add first 40 bytes of packet, then do 0xFF - chksm. If equals either byte 41 or 42 in packet, data valid.
  for(int i = 0; i < 2520; i = i+42){

    chksm = 0;

    for(int j = 0; j<40; j++){
      chksm = chksm + lidarData[i+j];
    }

    chksm = 0xFF - chksm;

    old_good_packet = good_packet;
    
    if(chksm == lidarData[i+40]){
      good_packet = good_packet + 1;
    }

    /*
    uint16_t rangeCount = 0;

    // Number of good packets increased, so create lidar distance data for the packet
    if(old_good_packet != good_packet){
      for (int j = i + 6; j < i + 40; j = j + 6) {
        int lidar_distance = (lidarData[j+1] << 8) + lidarData[j];
        
        laserScan.ranges.data[rangeCount] = float(lidar_distance / 1000.0); // needs to be in metres
        laserScan.ranges.size += 1;

        //int lidar_angle = ((lidarData[i+1] - 160) * 6) + ((j-6-i)/6);

        rangeCount += 1;
      
      }
    }
    */
    
  }

  

  //if received 60 good packets of data from LiDAR, calculate angles etc
  if(good_packet == 60){

    uint16_t rangeCount = 0;
    laserScan.ranges.size = 0;

    // now have full data packet, loop over each 42 byte packet
    for(int i = 0; i < 2520; i = i + 42){
      for (int j = i + 6; j < i + 40; j = j + 6) {
        int lidar_distance = (lidarData[j+1] << 8) + lidarData[j];
        
        laserScan.ranges.data[rangeCount] = float(lidar_distance / 1000.0); // needs to be in metres
        laserScan.ranges.size += 1;

        //int lidar_angle = ((lidarData[i+1] - 160) * 6) + ((j-6-i)/6);

        
        rangeCount += 1;
        
      }
    }
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // calculate rpm
    //rpm = ((lidarData[3] << 8) + lidarData[2]) / 10;
    
  }
  
  
   
  // reset all data to clear for next packet
  good_packet = 0;
  Sync0[0] = 0x00;
  Sync1[0] = 0x00;
  memset(lidarData, 0, sizeof(lidarData));
  memset(remainingData, 0, sizeof(remainingData));
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // Publish message
  rcl_ret_t rc = rcl_publish(&laserPublisher, &laserScan, NULL);



  //delay(100);

 
}
