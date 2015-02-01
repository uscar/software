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
#include <AP_Baro.h>

//Custom tools
#include "calibration.h"
#include "flight_control.h"
#include "flight_logic.h"
#include "routine.h"

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

Flight_Control* flight_control;
Flight_Logic* flight_logic;

void setup() {
  /** Baro initialization **/
  hal.gpio->pinMode(63, GPIO_OUTPUT);
  hal.gpio->write(63, 1);
  baro.init();
  baro.calibrate();

  flight_control = new Flight_Control();
  flight_logic = new Flight_Logic(flight_control);
}

void loop() {
  if(hal.console->available()) {
    int val = hal.console->read() - '0';
    if(val != -35) set_curr_routine(val);
  }
  flight_logic->ExecuteCurrRoutine();
}

void set_armed(bool armed) {
  flight_control->set_armed(armed);
}

void set_curr_routine(int routine_code) {
  flight_logic->set_curr_routine(routine_code);
}

AP_HAL_MAIN();
