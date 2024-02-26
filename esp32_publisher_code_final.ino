// Include necessary libraries
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h> // Include the IMU message type
#include <sensor_msgs/msg/magnetic_field.h> // Include the MagneticField message type
#include <std_msgs/msg/float32.h> // Include the Float32 message type

// Define variables and structures for Micro-ROS
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t mag_publisher; // Publisher for MagneticField messages
sensor_msgs__msg__MagneticField mag_msg; // MagneticField message
rcl_publisher_t voltage_publisher; // Publisher for voltage data
std_msgs__msg__Float32 voltage_msg; // Float32 message
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){}}

// Include your IMU libraries
#include "ICM20600.h"
#include "AK09918.h"
#include <Wire.h>
ICM20600 icm20600(true);
AK09918_err_type_t err;
int32_t x, y, z;
AK09918 ak09918;

// Conversion functions (if needed)
inline float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}

inline float MilliGToMeterPerSecond(float g)
{
  return g / 1000 * 9.80665;
}

inline float DegreesPerSecondToRadsPerSecond(float dps)
{
  return dps * 0.01745;
}

// Error handling function
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Timer callback function
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Get the current time in milliseconds
    uint32_t current_time_ms = millis();

    // Update the timestamp for IMU message
    imu_msg.header.stamp.sec = current_time_ms / 1000;  // Convert to seconds
    imu_msg.header.stamp.nanosec = (current_time_ms % 1000) * 1000000;  // Convert to nanoseconds

    imu_msg.angular_velocity.x = DegreesPerSecondToRadsPerSecond(icm20600.getGyroscopeX());
    imu_msg.angular_velocity.y = DegreesPerSecondToRadsPerSecond(icm20600.getGyroscopeY());
    imu_msg.angular_velocity.z = DegreesPerSecondToRadsPerSecond(icm20600.getGyroscopeZ());
    imu_msg.linear_acceleration.x = MilliGToMeterPerSecond(icm20600.getAccelerationX());
    imu_msg.linear_acceleration.y = MilliGToMeterPerSecond(icm20600.getAccelerationY());
    imu_msg.linear_acceleration.z = MilliGToMeterPerSecond(icm20600.getAccelerationZ());

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // Update the timestamp for MagneticField message
    mag_msg.header.stamp.sec = current_time_ms / 1000;  // Convert to seconds
    mag_msg.header.stamp.nanosec = (current_time_ms % 1000) * 1000000;  // Convert to nanoseconds

    err = ak09918.isDataReady();
    if (err == AK09918_ERR_OK) {
      err = ak09918.isDataSkip();
      if (err == AK09918_ERR_DOR) {
          //Serial.println(ak09918.strError(err));
      }
      err = ak09918.getData(&x, &y, &z);
      if (err == AK09918_ERR_OK) {
        mag_msg.magnetic_field.x = MicroTeslaToTesla(x); // Replace with actual magnetic field data
        mag_msg.magnetic_field.y = MicroTeslaToTesla(y); // Replace with actual magnetic field data
        mag_msg.magnetic_field.z = MicroTeslaToTesla(z); // Replace with actual magnetic field data

        RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
      } else {
          //Serial.println(ak09918.strError(err));
      }
    } else {
        //Serial.println(ak09918.strError(err));
    }
  }
}

// Implement your own function to read the voltage (replace this with your own logic)
float readVoltage() {
  // Implement your logic to read the voltage from a sensor or source
  // Return the voltage reading as a float
  float voltage = (float)analogRead(4) / 4096 * 14 * 30748.598/29300;
 
  if (voltage < 10.95){
    voltage = voltage + 0.35;
  }
  else if (voltage < 11.5){
    voltage = voltage + 0.3;
  }
  else if (voltage < 11.8){
    voltage = voltage + 0.25;
  }
  else if (voltage < 12.1){
    voltage = voltage + 0.2;
  }

  return voltage;
}

void setup() {
  Wire.begin();
  set_microros_transports();  // Initialize Micro-ROS transports
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  // gyroscope and accelerometer
  icm20600.initialize();
  // magnenometer
  err = ak09918.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

  err = ak09918.isDataReady();
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create publisher for IMU data on the imu/data_raw topic
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw")); // Set the topic name to imu/data_raw

  // Create publisher for magnetic field data on the imu/mag topic
  RCCHECK(rclc_publisher_init_default(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu/mag")); // Set the topic name to imu/mag

  // Create publisher for voltage data on the voltage_level topic
  RCCHECK(rclc_publisher_init_default(
    &voltage_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "voltage_level")); // Set the topic name to voltage_level

  // Create timer,
  const unsigned int timer_timeout = 10; // Adjust the timeout as needed
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize

 // Initialize IMU message
  imu_msg.header.frame_id.data = "imu_frame"; // Replace with your frame ID

  // Initialize MagneticField message
  mag_msg.header.frame_id.data = "imu_frame"; // Replace with your frame ID

  // Initialize voltage message
  voltage_msg.data = 0.0; // Initialize with the voltage value you want to send

  // You can also initialize the covariance matrix and orientation data for IMU if needed
}

void loop() {
  // ... (previous loop code remains the same)

  // Update the voltage reading (replace this with your voltage reading logic)
  float voltage_reading = readVoltage(); // Implement your own logic to read voltage


  // Update the voltage message with the new reading
  voltage_msg.data = voltage_reading;

  // Publish the voltage data
  RCSOFTCHECK(rcl_publish(&voltage_publisher, &voltage_msg, NULL));

  // Continue executing the executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}