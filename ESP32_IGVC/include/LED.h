#include <Adafruit_NeoPixel.h>
#include <ros.h> // ROS Libs
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#define LED_STRIPE_PIN 15
#define SAFETY_LED_PIN 22 

#define NUMPIXELS 39
#define NUM_PARTS 4
#define NUMPIXELS_PER_PART 10
#define BRIGHT_THRES 12

#define MIN_BRIGHT 0
#define MAX_BRIGHT 200
#define SAFETY_LED_BLINK_DELAY 250

// TODO: Check if FastLED gives better results.
Adafruit_NeoPixel pixels(NUMPIXELS, LED_STRIPE_PIN, NEO_GRB + NEO_KHZ400);
void set_colors(uint32_t color[NUM_PARTS]);
void autoset_brightness();
uint32_t Wheel(byte WheelPos);
void rainbowCycle();
//TODO: Write code for color change based on ROS msgs.

void blink_status_led();
void class_pred_cb(const std_msgs::Float32MultiArray& msg);
void path_path_cb(const std_msgs::Bool &msg);
void control_err_cb(const std_msgs::Byte &msg);
void braking_cb(const std_msgs::Bool& msg);