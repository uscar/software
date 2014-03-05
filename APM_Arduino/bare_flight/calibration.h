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

#define r_p      1.0
#define r_i      0.03
#define r_d      0.2
#define r_imax   20

#define p_p     1.0
#define p_i     0.03
#define p_d     0.2
#define p_imax  20

#define t_p    1.0
#define t_i    0.001
#define t_d    0.02
#define t_imax 0.5

#define y_p          .5
#define y_i          .6
#define y_d          .2
#define y_imax       10

#define error_scale .4

//low pass filter
#define update_hz 1000
#define cutout 40



//debugging setup
#define DEBUG_ALL false
#define DEBUG_FILTER false
#define DEBUG_PID false
#define DEBUG_ACTUAL true
#define DEBUG_MOTOR true
#define DEBUG_TIMER false
#define DEBUG_OUTPUT true
#define DEBUG_PWM false
#define DEBUG_CONTROL false
#define DEBUG_ENABLE false
#define DEBUG_PULSE true
//random

#define CNTRL_MIN 0
#define CNTRL_MAX 1000

//the min/max signals to which the output to motors are constrained
#define MOTOR_MIN 1150
#define MOTOR_MAX 1900
#define MOTOR_HOVER 1400
