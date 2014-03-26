///////////////////////////
//for the uncovered board//
///////////////////////////



//IMU consts

float offset_acc[3]  = {-.105,.800,1.0};
float scale_acc[3]   = {1,1,1};
float offset_gyr[3]  = {-.001,-.0001,.0002};
float scale_gyr[3]   = {1,1,1};


//Remote bounds

#define REMOTE_MAX 1900
#define REMOTE_MIN 1100
#define REMOTE_TRIM 1500

//Baro consts



//PID values

#define r_p      2.0
#define r_i      0.05
#define r_d      0.3
#define r_imax   5

#define p_p     2.0
#define p_i     0.05
#define p_d     0.3
#define p_imax  5

#define t_p    1.0
#define t_i    0.001
#define t_d    0.02
#define t_imax 0.5

#define y_p          .5
#define y_i          .6
#define y_d          .2
#define y_imax       10

#define error_scale .3

//low pass filter
#define update_hz 1000
#define cutout 40


#define GYR_ERR_SCALE 200

#define INV_ROLL false
#define INV_YAW true
#define INV_PITCH true


//debugging setup
#define DEBUG_ALL true
#define DEBUG_FILTER false
#define DEBUG_PID true
#define DEBUG_ACTUAL true
#define DEBUG_MOTOR false
#define DEBUG_TIMER false
#define DEBUG_OUTPUT false
#define DEBUG_PWM false
#define DEBUG_CONTROL false
#define DEBUG_ENABLE false
#define DEBUG_PULSE false
#define DEBUG_CNTRL false
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
#define YAW_SCALE 100


#define RPY_RC_MIN 1100
#define RPY_RC_MAX 1900
#define RPY_RC_TRIM 1500

//the dynamic range given (zeor to max, min to zero) for cntrl values
#define CNTRL_RANGE 1000
//the z componenet of the to-be normalized control vector (a large value dampens controller effect)
#define CNTRL_Z_COMP 5
