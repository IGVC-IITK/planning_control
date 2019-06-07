#include <Arduino.h>

#include <ros.h> // ROS Libs
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include "RFReceiver.h" // RF Receiver
#include "LED.h"		// LED Strip

// #define DEBUG
#define KILL_PIN LED_BUILTIN

#define MODE_STOP -1
#define MODE_MANUAL 0
#define MODE_AUTO 1

#define MAX_LIN_VEL 2.0
#define MAX_ANG_VEL 4.0
#define INP_ANG_VEL 0.5

// ROS Global Variables
ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;
std_msgs::Byte ros_op_mode;
ros::Publisher velocity_pub("ros0xrobot/cmd_vel", &cmd_vel);
ros::Publisher ros_mode_pub("op_mode", &ros_op_mode);
ros::Subscriber<std_msgs::Float32MultiArray> cp_sub("fast_scnn/class_predictions", &class_pred_cb);
ros::Subscriber<std_msgs::Bool> path_plan_sub("/replan_path", &path_path_cb);
ros::Subscriber<std_msgs::Byte> control_err_sub("/control_err", &control_err_cb);
ros::Subscriber<std_msgs::Bool> braking_sub("/brake", &braking_cb);

// Other Global Vars
unsigned long loop_prev_millis = 0;
const long loop_interval = 50; // Sets the publishing rate interval = 1000 / pub_rate (Hz)
unsigned long rainbow_prev_millis = 0;
const long rainbow_interval = 50; // Sets the publishing rate interval = 1000 / pub_rate (Hz)
bool toggle = true;

// Uncomment to serial print debug info.
// #define DEBUG

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

float mapf(long x, long in_min, long in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void init_ch_pins()
{
	for (uint8_t i = 0; i < NUM_CH; i++)
		pinMode(ch_pin[i], INPUT);
}

void read_all_ch()
{
	ch_val[OPMODE_CH_IDX] = pulseIn(ch_pin[OPMODE_CH_IDX], HIGH, 30000);
	if (ch_val[OPMODE_CH_IDX] < ch_midl[OPMODE_CH_IDX])
	{
		// Stop mode.
		// TODO:
		for (uint8_t i = 0; i < OPMODE_CH_IDX; i++)
			ch_val[i] = ch_mid[i];
		for (uint8_t i = OPMODE_CH_IDX + 1; i < NUM_CH; i++)
			ch_val[i] = ch_mid[i];
	}
	else if (ch_val[OPMODE_CH_IDX] >= ch_midl[OPMODE_CH_IDX] && ch_val[OPMODE_CH_IDX] <= ch_midh[OPMODE_CH_IDX])
	{
		// Manual mode.
		for (uint8_t i = 0; i < OPMODE_CH_IDX; i++)
			ch_val[i] = pulseIn(ch_pin[i], HIGH, 30000);
		for (uint8_t i = OPMODE_CH_IDX + 1; i < NUM_CH; i++)
			ch_val[i] = pulseIn(ch_pin[i], HIGH, 30000);
	}
	else
	{
		// AUTONOMOUS MODE
		ch_val[LED_CH_IDX] = pulseIn(ch_pin[LED_CH_IDX], HIGH, 30000);
		//TODO: ??
	}
}

void print_all_ch()
{
	for (uint8_t i = 0; i < OPMODE_CH_IDX; i++)
		Serial.printf("ch[%1d] %04d | ", i, ch_val[i]);
	Serial.printf("** ");
	for (uint8_t i = OPMODE_CH_IDX; i < NUM_CH; i++)
		Serial.printf("ch[%1d] %04d | ", i, ch_val[i]);
	Serial.println();
}

void pub_cmd_vel()
{
	/* Decoding Max Speed and angular rate */
	float max_lin_vel = mapf(ch_val[MAXVEL_CH_IDX], ch_min[MAXVEL_CH_IDX] - CH_VAL_THRES, ch_max[MAXVEL_CH_IDX] + CH_VAL_THRES, 0, MAX_LIN_VEL);
	float max_ang_vel = mapf(ch_val[MAXVEL_CH_IDX], ch_min[MAXVEL_CH_IDX] - CH_VAL_THRES, ch_max[MAXVEL_CH_IDX] + CH_VAL_THRES, 0, MAX_ANG_VEL);
	/* Decoding Current Linear and Agular Speed*/
	float v_lin_x = 0;
	if (ch_val[LIN_VEL_CH_IDX] > ch_midl[LIN_VEL_CH_IDX] && ch_val[LIN_VEL_CH_IDX] < ch_midh[LIN_VEL_CH_IDX])
		v_lin_x = 0;
	else
		v_lin_x = mapf(ch_val[LIN_VEL_CH_IDX], ch_min[LIN_VEL_CH_IDX] - CH_VAL_THRES, ch_max[LIN_VEL_CH_IDX] + CH_VAL_THRES, -max_lin_vel, max_lin_vel);

	float v_ang_z = 0;
	if (ch_val[ANG_VEL_CH_IDX] > ch_midl[ANG_VEL_CH_IDX] && ch_val[ANG_VEL_CH_IDX] < ch_midh[ANG_VEL_CH_IDX])
		v_ang_z = 0;
	else
		v_ang_z = mapf(ch_val[ANG_VEL_CH_IDX], ch_min[ANG_VEL_CH_IDX] - CH_VAL_THRES, ch_max[ANG_VEL_CH_IDX] + CH_VAL_THRES, -max_ang_vel, max_ang_vel);
	/* Decoding inplace turn */
	if (ch_val[INP_ANG_VEL_CH_IDX] > (ch_mid[INP_ANG_VEL_CH_IDX] + ch_max[INP_ANG_VEL_CH_IDX]) / 2)
	{
		v_ang_z = INP_ANG_VEL;
		v_lin_x = 0;
	}
	else if (ch_val[INP_ANG_VEL_CH_IDX] < (ch_mid[INP_ANG_VEL_CH_IDX] + ch_min[INP_ANG_VEL_CH_IDX]) / 2)
	{
		v_ang_z = -INP_ANG_VEL;
		v_lin_x = 0;
	}
	/* Put the values into ROS messages */
	// TODO: Make this work (Not working now)
	v_lin_x = floor(100 * v_lin_x) / 100; // Set precision to two decimals
	v_ang_z = floor(100 * v_ang_z) / 100; // Set precision to two decimals
	cmd_vel.linear.x = v_lin_x;
	cmd_vel.angular.z = v_ang_z;
	velocity_pub.publish(&cmd_vel);
}

void set_colors(uint32_t color[NUM_PARTS])
{
	for (size_t i = 0; i < NUM_PARTS; i++)
		for (int j = i * NUMPIXELS_PER_PART; j < (i + 1) * NUMPIXELS_PER_PART; j++)
			pixels.setPixelColor(j, color[i]);
	pixels.show();
}

void autoset_brightness()
{
	static uint8_t curr_brightness = MAX_BRIGHT;
	static uint8_t next_brightness = MAX_BRIGHT;
	next_brightness = map(ch_val[LED_CH_IDX], ch_min[LED_CH_IDX] - CH_VAL_THRES, ch_max[LED_CH_IDX] + CH_VAL_THRES, MIN_BRIGHT, MAX_BRIGHT);
	if (abs(next_brightness - curr_brightness) > BRIGHT_THRES)
	{
		pixels.setBrightness(next_brightness);
		curr_brightness = next_brightness;
	}
}

int8_t autoset_mode()
{
	static int8_t curr_mode = MODE_STOP;
	static int8_t next_mode = MODE_STOP;

	if (ch_val[OPMODE_CH_IDX] < ch_midl[OPMODE_CH_IDX])
		next_mode = MODE_STOP;
	else if (ch_val[OPMODE_CH_IDX] >= ch_midl[OPMODE_CH_IDX] && ch_val[OPMODE_CH_IDX] <= ch_midh[OPMODE_CH_IDX])
		next_mode = MODE_MANUAL;
	else
		next_mode = MODE_AUTO;

	if (next_mode != curr_mode)
	{
		curr_mode = next_mode;
		ros_op_mode.data = curr_mode;
		ros_mode_pub.publish(&ros_op_mode);
	}
	return curr_mode;
}

uint32_t Wheel(byte WheelPos)
{
	if (WheelPos < 85)
	{
		return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
	}
	else if (WheelPos < 170)
	{
		WheelPos -= 85;
		return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	else
	{
		WheelPos -= 170;
		return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle()
{ // modified from Adafruit example to make it a state machine
	static uint16_t j = 0;
	for (int i = 0; i < pixels.numPixels(); i++)
	{
		pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
	}
	pixels.show();
	j = j + 10;
	if (j >= 256 * 5)
		j = 0;
	rainbow_prev_millis = millis(); // time for next change to the display
}

void blink_status_led()
{
	static long prev_millis = 0;
	if (millis() - prev_millis > SAFETY_LED_BLINK_DELAY)
	{
		digitalWrite(SAFETY_LED_PIN, HIGH - digitalRead(SAFETY_LED_PIN));
		prev_millis = millis();
	}
}

void class_pred_cb(const std_msgs::Float32MultiArray &msg)
{
	int lane = msg.data[2] * 100;
	lane = lane > 10 ? 10 : lane;

	// TODO: Use pixels.fill()

	for (int i = NUMPIXELS - 1; i > NUMPIXELS - 1 - (lane) / 10.0 * NUMPIXELS_PER_PART; i--)
		pixels.setPixelColor(i, pixels.Color(255, 255, 255));
	for (int i = NUMPIXELS - 1 - (10.0 - lane) / 10.0 * NUMPIXELS_PER_PART; i > NUMPIXELS - 1 - NUMPIXELS_PER_PART; i--)
		pixels.setPixelColor(i, pixels.Color(0, 0, 255));

	pixels.show();
	// for (int i = 3 * NUMPIXELS_PER_PART + 4.0 / 6.0 * NUMPIXELS_PER_PART - 2; i < 4 * NUMPIXELS_PER_PART; i++)
	// 	pixels.setPixelColor(i, pixels.Color(255, 255, 255));
	// for (int i = 3 * NUMPIXELS_PER_PART - 2 i < 3 * NUMPIXELS_PER_PART + (10.0 - lane) / 10.0 * NUMPIXELS_PER_PART - 1; i++)
	// 	pixels.setPixelColor(i, pixels.Color(0, 0, 255));
}

void path_path_cb(const std_msgs::Bool &msg)
{
	uint32_t color;
	if (msg.data == true)
		color = pixels.Color(255, 0, 255);
	else
		color = pixels.Color(255, 0, 60);

	//TODO: Use pixels.fill()
	for (int i = NUMPIXELS - 1 - NUMPIXELS_PER_PART; i > NUMPIXELS - 1 - 2 * NUMPIXELS_PER_PART; i--)
		pixels.setPixelColor(i, color);
	pixels.show();
}

void control_err_cb(const std_msgs::Byte &msg)
{
	uint32_t color = pixels.Color(255 - msg.data, msg.data, 0);
	for (int i = NUMPIXELS - 1 - 2 * NUMPIXELS_PER_PART; i > NUMPIXELS - 1 - 3 * NUMPIXELS_PER_PART; i--)
		pixels.setPixelColor(i, color);
	pixels.show();
}

void braking_cb(const std_msgs::Bool& msg)
{
	uint32_t color = msg.data == true ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0);
	for (int i = NUMPIXELS - 1 - 3 * NUMPIXELS_PER_PART; i > -1; i--)
		pixels.setPixelColor(i, color);
	pixels.show();
}

void setup()
{
// TODO: put ifdef
#ifdef DEBUG
	Serial.begin(57600);
#endif
	init_ch_pins(); // Init all channel pins to input.
	pixels.begin(); // Begin LED Strip

	nh.initNode(); // ROS init and advertize.
	nh.advertise(velocity_pub);
	nh.advertise(ros_mode_pub);

	nh.subscribe(cp_sub);
	nh.subscribe(path_plan_sub);
	nh.subscribe(control_err_sub);
	nh.subscribe(braking_sub);

	pinMode(KILL_PIN, OUTPUT);		 // KILL PIN
	pinMode(SAFETY_LED_PIN, OUTPUT); // KILL PIN
}

void loop()
{
	unsigned long curr_millis = millis();
	read_all_ch();
#ifdef DEBUG
	print_all_ch();
#endif
	autoset_brightness();
	int8_t op_mode = autoset_mode();
	if (op_mode == MODE_STOP)
	{
		digitalWrite(KILL_PIN, HIGH);
		digitalWrite(SAFETY_LED_PIN, LOW);
	}
	else if (op_mode == MODE_AUTO)
	{
		digitalWrite(KILL_PIN, LOW);
		blink_status_led();
		uint32_t colors[NUM_PARTS + 1] = {pixels.Color(0, 0, 0), pixels.Color(63, 172, 0), pixels.Color(255, 0, 60), pixels.Color(0, 0, 255), pixels.Color(255, 255, 255)};
		for (int i = 0; i < NUMPIXELS_PER_PART - 1; i++)
			pixels.setPixelColor(i, colors[0]);
		for (int i = NUMPIXELS_PER_PART - 2; i < 2 * NUMPIXELS_PER_PART - 1; i++)
			pixels.setPixelColor(i, colors[1]);
		for (int i = 2 * NUMPIXELS_PER_PART - 2; i < 3 * NUMPIXELS_PER_PART - 1; i++)
			pixels.setPixelColor(i, colors[2]);

		pixels.show();
	}
	else if (op_mode == MODE_MANUAL)
	{
		digitalWrite(KILL_PIN, LOW);
		digitalWrite(SAFETY_LED_PIN, HIGH);
		if (curr_millis - rainbow_prev_millis > rainbow_interval)
			rainbowCycle();
	}

	// Publish at 20 Hz rate
	if (curr_millis - loop_prev_millis >= loop_interval)
	{
		if (op_mode == MODE_MANUAL)
			pub_cmd_vel();
		nh.spinOnce();
		loop_prev_millis = curr_millis;
	}
}