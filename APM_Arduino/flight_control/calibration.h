///////////////////////////
//for the uncovered board//
///////////////////////////

//Remote bounds

#define REMOTE_MAX 1900
#define REMOTE_MIN 1100
#define REMOTE_TRIM 1500

//was .5

//low pass filter
#define update_hz 1000
#define cutout 40

#define INV_ROLL false
#define INV_YAW true
#define INV_PITCH true

//debugging setup
#define DEBUG_ALL false
#define DEBUG_FILTER false
#define DEBUG_PID true
#define DEBUG_ACTUAL true
#define DEBUG_MOTOR true
#define DEBUG_TIMER false
#define DEBUG_OUTPUT false
#define DEBUG_PWM false
#define DEBUG_CONTROL false
#define DEBUG_ENABLE false
#define DEBUG_PULSE false
#define DEBUG_CNTRL false
#define DEBUG_GYRERR true
#define DEBUG_ACCERR true
//random


//the min/max signals to which the output to motors are constrained
#define M_SERV_MIN 0
#define M_SERV_MAX 1000

#define THROTTLE_SIG_MAX 1000
#define THROTTLE_SIG_MIN 0

#define DIR_SIG_MIN -1000
#define DIR_SIG_MAX 1000

#define MOTOR_MIN 1150
#define MOTOR_MAX 1900
#define MOTOR_HOVER 1400

//how much is yaw rate weighted relative to acc.

#define RPY_RC_MIN 1100
#define RPY_RC_MAX 1900
#define RPY_RC_TRIM 1500

//the z componenet of the to-be normalized control vector (a large value dampens controller effect)
#define CNTRL_Z_COMP 3
