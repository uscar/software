#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

AP_MotorsQuad   motors(&rc1, &rc2, &rc3, &rc4);

void setup(){
    
}
