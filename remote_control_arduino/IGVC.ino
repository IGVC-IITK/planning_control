#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

// IMPORTANT: ROS nodehandle DOES NOT work when Uncommented
// Use only while debugging to see values on all the channels
// #define SERIAL_DEBUG

// These are the pins to which the respective channels etc are connected
#define CH6 6
#define CH5 7
#define CH4 8
#define CH3 9
#define CH2 10
#define CH1 11
#define BIND 5
#define ESTOP 4
#define STATUS_LED 13

// ---------------------------------------------------------------
//  Vaiable |     Function     |  Location on RC  |Range (approx)|
// ---------------------------------------------------------------
//  ch1     |   Inplace Turn   |  Right Left/Right|  995 - 2000  |
//  ch2     |  Move Front/Back |  Right Up/Down   |  993 - 2000  |
//  ch3     |Inc/Dec max speed |  Left Up/Down    |  993 - 2000  |
//  ch4     |   Radial Turn    |  Left Left/Right |  984 - 1980  |
//  ch5     |  Mode of Operate |  Left Top Wheel  |  984 - 1980  |
//  ch6     |      E Stop      |  Right Top Wheel |  984 - 1980  |
// ---------------------------------------------------------------

// Stores the values of all channels
float ch6;
float ch5;
float ch4;
float ch3;
float ch2;
float ch1;

// ROS Stuff
ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;
std_msgs::Bool is_manual;
ros::Publisher velocity_pub("ros0xrobot/cmd_vel", &cmd_vel);
ros::Publisher mode_pub("manual_mode", &is_manual);

void setup() {
  
  nh.initNode();
  nh.advertise(velocity_pub);
  nh.advertise(mode_pub);
  
  #ifdef SERIAL_DEBUG
  Serial.begin(9600);
  #endif
  
  init_channels();
  // For Flashing LED
  pinMode(STATUS_LED, OUTPUT);
  // For E stop
  pinMode(ESTOP, OUTPUT);
}

void loop() {

  read_all_channels();
  #ifdef SERIAL_DEBUG
  print_all_channels();
  #endif
  channel_to_ros();
  
  nh.spinOnce();
  delay(1);
}

// Initialize the pinModes
void init_channels(){
  pinMode(CH6, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH1, INPUT);
}

// Reads all the channels
void read_all_channels(){
  int temp;
  temp = pulseIn(CH6, HIGH, 30000);
  if(temp)
    ch6 = temp;
  temp = pulseIn(CH5, HIGH, 30000);
  if(temp)
    ch5 = temp; 
  temp = pulseIn(CH4, HIGH, 30000);
  if(temp)
    ch4 = temp;
  temp = pulseIn(CH3, HIGH, 30000);
  if(temp)
    ch3 = temp;
  temp = pulseIn(CH2, HIGH, 30000);
  if(temp)
    ch2 = temp;
  temp = pulseIn(CH1, HIGH, 30000);
  if(temp)
    ch1 = temp; 
}

// Prints all the Channels
void print_all_channels(){
  Serial.print("ch1 " );
  Serial.print(ch1);
  Serial.print("\tch2 " );
  Serial.print(ch2);
  Serial.print("\tch3 " );
  Serial.print(ch3);
  Serial.print("\tch4 " );
  Serial.print(ch4);
  Serial.print("\tch5 " );
  Serial.print(ch5);
  Serial.print("\tch6 " );
  Serial.println(ch6);
}

// Creates ROS messages from channel info
void channel_to_ros(){
  /* Decoding Max Speed and angular rate */
  // Limiting the range from 1000 to 2000
  ch3 = ch3 > 2000 ? 2000 : ch3;
  ch3 = ch3 < 1000 ? 1000 : ch3;
  float max_vel = ((ch3 - 1000)/1000.0) * 2.0;
  float max_ang = ((ch3 - 1000)/1000.0) * 1.0;

  /* Decoding forward/backward movement */
  // Limiting the range from 1000 to 2000
  ch2 = ch2 > 2000 ? 2000 : ch2;
  ch2 = ch2 < 1000 ? 1000 : ch2;
  float v_linear_x = 0;
  if(ch2 > 1450 && ch2 < 1550)    
    v_linear_x = 0;                             // No movement
  else if(ch2 > 1550)
    v_linear_x = ((ch2 - 1550)/450.0) * max_vel;  // Go Forward
  else
    v_linear_x = ((ch2 - 1450)/450.0) * max_vel;  // Go Backward

  /* Decoding Radial Turn */
  // Limiting the range from 1000 to 2000
  ch4 = ch4 > 2000 ? 2000 : ch4;
  ch4 = ch4 < 1000 ? 1000 : ch4;
  float v_angle_z = 0;
  if(ch4 > 1450 && ch4 < 1550)    
    v_angle_z = 0;                             // No movement
  else if(ch4 > 1550)
    v_angle_z = (-(ch4 - 1550)/450.0) * max_ang;  // Go Forward
  else
    v_angle_z = (-(ch4 - 1450)/450.0) * max_ang;  // Go Backward

  /* Decoding inplace turn */
  // Limiting the range from 1000 to 2000
  ch1 = ch1 > 2000 ? 2000 : ch1;
  ch1 = ch1 < 1000 ? 1000 : ch1;
  if(ch1 > 1450 && ch1 < 1550 && v_angle_z == 0)
    v_angle_z = 0;                             // No movement
  else if(ch1 > 1550 && v_angle_z == 0){
    v_angle_z = (-(ch1 - 1550)/450.0) * max_ang;  // Turn Left
    v_linear_x = 0;
  }
  else if(ch1 < 1450 && v_angle_z == 0){
    v_angle_z = (-(ch1 - 1450)/450.0) * max_ang;  // Turn Right
    v_linear_x = 0;
  }

  /* Autonomous or manual mode */
  static bool prev_manual_mode = false;
  bool manual_mode = true;
  if(ch5 > 1550)
    manual_mode = true;
  else if (ch5 < 1450)
    manual_mode = false;
  
  /* E STOP */
  if(ch6 > 1550){
    bool estop = true;
    digitalWrite(ESTOP, HIGH);
    Serial.println("Stop");
  }
  else if (ch6 < 1450){
    bool estop = false;
    digitalWrite(ESTOP, LOW);
  }
  // Autonomous Mode then blink else constant
  static bool state = LOW;
  static long prev_millis = 0;
  if(!manual_mode){
    if(millis() - prev_millis > 500){
      prev_millis = millis();
      state = !state;
      digitalWrite(13, state);
    }
  }
  else{
    prev_millis = millis();
    digitalWrite(13, HIGH);
  }
  
  // Get the values into ROS messages.
  cmd_vel.linear.x = v_linear_x;
  cmd_vel.angular.z = v_angle_z;
  is_manual.data = manual_mode;
  
  /* Conditions for publishing to ROS */
  if(manual_mode){
    velocity_pub.publish(&cmd_vel);
  }
  if(manual_mode != prev_manual_mode){
    mode_pub.publish(&is_manual);
  }
  prev_manual_mode = manual_mode;
}

