/*
 * ESP32 micro-ROS Rover
 *  - sensor_msgs/Imu  yayini  -> /imu/data_raw   (MPU6050)
 *  - geometry_msgs/Twist abonesi -> /cmd_vel     -> motor surus
 *  - Seri (USB) transport, Pi'de micro-ROS agent
 *  Core 3.x, Board: ESP32 Dev Module, Partition: Huge APP
 */
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <Wire.h>

#define MPU_ADDR 0x68
#define SDA_PIN  21
#define SCL_PIN  22
#define DEG2RAD  0.01745329252f

#define LEFT_IN1  25
#define LEFT_IN2  26
#define RIGHT_IN1 19
#define RIGHT_IN2 23
#define SPEED_MAX 180

#define LIN_GAIN  300.0f
#define ANG_GAIN  120.0f
#define LED_PIN   2

rcl_publisher_t    imu_pub;
rcl_subscription_t cmd_sub;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist  twist_msg;
rclc_executor_t  executor;
rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rcl_timer_t      timer;

static char imu_frame_id[] = "imu_link";
unsigned long last_cmd_ms = 0;
const unsigned long CMD_TIMEOUT = 500;

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
void error_loop(){ while(1){ digitalWrite(LED_PIN,!digitalRead(LED_PIN)); delay(100);} }

void setMotor(int pinA, int pinB, int spd) {
  spd = constrain(spd, -SPEED_MAX, SPEED_MAX);
  if      (spd > 0) { analogWrite(pinA, spd); analogWrite(pinB, 0); }
  else if (spd < 0) { analogWrite(pinA, 0);   analogWrite(pinB, -spd); }
  else              { analogWrite(pinA, 0);   analogWrite(pinB, 0); }
}
void moveRobot(int l, int r) {
  setMotor(LEFT_IN1, LEFT_IN2, l);
  setMotor(RIGHT_IN1, RIGHT_IN2, r);   // sag ters donuyorsa: ...RIGHT_IN2, -r);
}
void stopRobot(){ moveRobot(0,0); }

void mpuBegin(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
}
void readIMU(float &ax,float &ay,float &az,float &gx,float &gy,float &gz){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  uint8_t b[14]; for(int i=0;i<14;i++) b[i]=Wire.read();
  int16_t rax=(b[0]<<8)|b[1], ray=(b[2]<<8)|b[3], raz=(b[4]<<8)|b[5];
  int16_t rgx=(b[8]<<8)|b[9], rgy=(b[10]<<8)|b[11], rgz=(b[12]<<8)|b[13];
  ax=rax/16384.0f*9.81f; ay=ray/16384.0f*9.81f; az=raz/16384.0f*9.81f;
  gx=rgx/131.0f*DEG2RAD; gy=rgy/131.0f*DEG2RAD; gz=rgz/131.0f*DEG2RAD;
}

void timer_callback(rcl_timer_t * t, int64_t lct){
  (void)lct; if(t==NULL) return;
  float ax,ay,az,gx,gy,gz; readIMU(ax,ay,az,gx,gy,gz);
  int64_t ns = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec=(int32_t)(ns/1000000000LL);
  imu_msg.header.stamp.nanosec=(uint32_t)(ns%1000000000LL);
  imu_msg.linear_acceleration.x=ax; imu_msg.linear_acceleration.y=ay; imu_msg.linear_acceleration.z=az;
  imu_msg.angular_velocity.x=gx; imu_msg.angular_velocity.y=gy; imu_msg.angular_velocity.z=gz;
  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

void cmd_vel_callback(const void * msgin){
  const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist *)msgin;
  float v = m->linear.x;
  float w = m->angular.z;
  int left  = (int)(LIN_GAIN * v - ANG_GAIN * w);
  int right = (int)(LIN_GAIN * v + ANG_GAIN * w);
  moveRobot(left, right);
  last_cmd_ms = millis();
}

void setup(){
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  pinMode(LEFT_IN1,OUTPUT); pinMode(LEFT_IN2,OUTPUT);
  pinMode(RIGHT_IN1,OUTPUT); pinMode(RIGHT_IN2,OUTPUT);
  stopRobot();

  Wire.begin(SDA_PIN, SCL_PIN);
  mpuBegin();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_rover_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  rmw_uros_sync_session(1000);

  imu_msg.header.frame_id.data     = imu_frame_id;
  imu_msg.header.frame_id.size     = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;

  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop(){
  if (millis() - last_cmd_ms > CMD_TIMEOUT) stopRobot();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
  delay(5);
}