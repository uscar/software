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

#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>

#include <Filter.h>
#include <LowPassFilter2p.h>

#include <AP_Notify.h>

#include <PID.h> //pid controller
#include <AC_PID.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

Flight_Control::Flight_Control(RC_Channel* rc1, RC_Channel* rc2, RC_Channel* rc3, RC_Channel* rc4){
	this->rc1 = rc1;
	this->rc2 = rc2;
	this->rc3 = rc3;
	this->rc4 = rc4;
	AC_PID pid (h_p, h_i, h_d, h_imax);
}

void Flight_Control::takeoff(){
	
}