#include <AP_ADC_HIL.h>
#include <AP_ADC.h>


#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_InertialSensor.h>
#include <GCS_MAVLink.h>

#include <AP_Notify.h>
#include "uscar_printing.h";

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
AP_InertialSensor_MPU6000 ins;


void setup(void)
{
    // HAL will start serial port at 115200.
  hal.console->println_P(PSTR("Starting!"));
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40,1);
  ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_100HZ);
}

void loop(void)
{
  hal.scheduler->delay(10);
  printIMU();
}

void printIMU(){
  Vector3f acc;
  float acc_amp;
  Vector3f gyr;
  
  ins.update();
  acc = ins.get_accel();
  gyr = ins.get_gyro();
  acc_amp = acc.length();
  char str[100];
  hal.console->printf("acc:");
  printv3f(acc,hal);
  hal.console->printf("  gyr:");
  printv3f(gyr,hal);
  hal.console->printf("\n");
}



AP_HAL_MAIN();
