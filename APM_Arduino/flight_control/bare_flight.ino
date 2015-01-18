/***********************************NOTES***************************************

RC_Channel:
radio_in = the pwm value given in
control_in = the range mapped output value 
.output() doesn't need to be called (it is for reading from a hal rc
radio_max = max input value
radio_min = min input value
.set_range(int min, int max) : min = min output, max = max output
AP_Motors:
uses the radio_in     
 *******************************************************************************/


//General board dependencies
#include <AP_ADC_HIL.h>
#include <AP_ADC.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

//IMU sensor dependencies
#include <AP_InertialSensor.h>
#include <GCS_MAVLink.h>
#include <AP_Notify.h>

//Motor output dependencies
#include <RC_Channel.h> 
#include <AP_Motors.h>
#include <AP_Curve.h>

//Algorithm tools
#include <LowPassFilter2p.h>
#include <AC_PID.h>

//Custom tools
#include "calibration.h"
#include "flight_control.h"

//board MUST be declared like this for libraries to function (they use extern)
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_InertialSensor_MPU6000 ins;

//setup the filters for removing noise from acc.

//vector command storage
Vector3f cntrl_up;

//non-vector command storage
float cntrl_yaw;
float old_cntrl_yaw;
int cntrl_throttle;

//used for calculating update rate, time elapsed etc
int timestamp;
int t0;
int counter = 0;

bool armed;
//reads remote control inputs and sets c vals
Flight_Control* flight_control;

void read_rc_inputs(){

  int roll        = hal.rcin->read(CH_1);
  int pitch       = hal.rcin->read(CH_2);
  int throttle    = hal.rcin->read(CH_3);
  int yaw         = hal.rcin->read(CH_4);

  if (yaw < REMOTE_MIN + 50 && throttle < REMOTE_MIN +50){
    armed = true;
  }

  if (yaw > REMOTE_MAX - 50 && throttle < REMOTE_MIN+50){
    armed = false;
  }


  float range = REMOTE_MAX-REMOTE_MIN;

    //calculate control value from input value
  cntrl_up.x = (INV_PITCH?-1:1)*(pitch - REMOTE_TRIM)*2.0/range;
  cntrl_up.y = (INV_ROLL?-1:1)*(roll - REMOTE_TRIM)*2.0/range;
  cntrl_up.z = CNTRL_Z_COMP; //dampens the control.

  cntrl_up = cntrl_up.normalized();
  cntrl_yaw  = (INV_YAW?-1:1)*(yaw - REMOTE_TRIM)*2.0/range;
  cntrl_throttle = (throttle - REMOTE_MIN);
}   

//linearly maps a value within one range to another range
int lin_map(int value, int min_v, int max_v, int min_o, int max_o) {
  return (int)((value-min_v)*((float)(max_o-min_o)/(float)(max_v-min_v))+min_o);
}

//called once on reset
void setup(void) {
  flight_control = new Flight_Control();
  flight_control->arm(true);
}

//called iteratively
void loop(void) {
  //for periodic outputs
  counter++;
  counter %= 50;
  if(armed) {
    flight_control->arm(true);
  }
  else {
    flight_control->arm(false);
  }
  read_rc_inputs();
  
  flight_control->execute(cntrl_up, cntrl_throttle, cntrl_yaw);
  //TODO:provide user interface for calibrating the accelerometer
  //also same thing for pid stuff
}

//calls setup and loop appropriately
  AP_HAL_MAIN();
