#define NUM_CH 8
#define CH_VAL_THRES 20
#define CH_VEL_THRES 40

#define LED_CH_IDX 4
#define OPMODE_CH_IDX 5
#define MAXVEL_CH_IDX 2
#define LIN_VEL_CH_IDX 1
#define ANG_VEL_CH_IDX 3
#define INP_ANG_VEL_CH_IDX 0

// Fixed & Read-only variables.
const static uint8_t ch_pin[NUM_CH]   = {  34,   35,   32,   33,   25,   26,   27,   14};
const static uint16_t ch_min[NUM_CH]  = { 988,  988,  988,  988,  988,  988,  988,  988};
const static uint16_t ch_midl[NUM_CH] = {1475, 1468, 1487, 1475, 1475, 1475, 1475, 1475};
const static uint16_t ch_mid[NUM_CH]  = {1500, 1493, 1512, 1500, 1500, 1500, 1500, 1500};
const static uint16_t ch_midh[NUM_CH] = {1525, 1518, 1537, 1525, 1525, 1525, 1525, 1525};
const static uint16_t ch_max[NUM_CH]  = {2013, 2013, 2010, 2013, 2013, 2013, 2013, 2013};

// Global variables.
uint16_t ch_val[NUM_CH]{};

// Function prototypes.
void init_ch_pins();
void read_all_ch();  // Return Mode of Operation
void print_all_ch();
void pub_cmd_vel();
int8_t autoset_mode();
float mapf(long x, long in_min, long in_max, float out_min, float out_max);