#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

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

#include <RangeFinder.h>
#include "constants.h"

class Flight_Control {
public:
	Flight_Control(RC_Channel* rc1, RC_Channel* rc2, RC_Channel* rc3, RC_Channel* rc4);
	void arm();
	void execute(Vector3f up, float throttle, float yaw);
	void takeoff();
	void altitudeHold();
	void landing();
private:
	RC_Channel* rc1, rc2, rc3, rc4;
	AC_PID* pid;
	RangeFinder ultrasonic;
	float _height;
};

#endif