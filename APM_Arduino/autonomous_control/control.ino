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
#include "takeoff.h"

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_InertialSensor_MPU6000 ins;

Flight_Control* flight_control;

void setup() {
  flight_control = new Flight_Control();
  flight_control->arm(true);
}

void loop() {

}

AP_HAL_MAIN();