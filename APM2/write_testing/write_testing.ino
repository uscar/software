#include <AP_ADC_HIL.h>
#include <AP_ADC.h>


#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>



const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_ADC_HIL adc;

void setup(void)
{
    // HAL will start serial port at 115200.
    hal.console->println_P(PSTR("Starting!"));
    adc.Init();
}

void loop(void)
{
  hal.scheduler->delay(100);
  for(int i = 0;i<8;i++){
    hal.console->printf(" %f ",adc.Ch(i));
  }
  hal.console->printf("\n");
}

AP_HAL_MAIN();
