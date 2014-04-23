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

extern AP_HAL::HAL& hal;
extern AP_InertialSensor_MPU6000 ins;

struct kPID{
	float P;
	float I;
	float D;
	float Imax;
};

enum DEBUG
{
	DEBUG_ALL,
	DEBUG_GYRERR,
	DEBUG_ACCERR,
	DEBUG_ACTUAL,
	DEBUG_CONTROL,
	DEBUG_PID,
	DEBUG_CNTRL,
	DEBUG_PULSE,
	DEBUG_OUTPUT,
	DEBUG_MOTOR,
	DEBUG_ENABLE,
	DEBUG_PWM
};

class Flight_Control {
public:
	Flight_Control();
	void arm(bool armed);
	void execute(Vector3f cntrl_up, float cntrl_throttle, float cntrl_yaw = 0);

	void setRollPID(kPID rPid);
	kPID getRollPID();
	void setPitchPID(kPID pPid);
	kPID getPitchPID();
	void setYawPID(kPID yPid);
	kPID getYawPID();

	void setGyrFactor(int f);
	int getGyrFactor();

private:
	bool armed;
	RC_Channel m_roll(2), m_pitch(3), m_throttle(1), m_yaw(4);
	AP_MotorsQuad motors(&m_roll,&m_pitch, &m_throttle, &m_yaw);
	AC_PID* pid;
	RangeFinder ultrasonic;

	kPID rPid  = {0.125,0.00,0.008,4};
	kPID pPid = {0.125,0.00,0.008,4};
	kPID yPid   = {5.000,0.005,0.000,8};

	AC_PID pid_roll     (rPid.P,rPid.I,rPid.D,rPid.Imax);
	AC_PID pid_pitch    (pPid.P,pPid.I,pPid.D,pPid.Imax);
	AC_PID pid_throttle (t_p,t_i,t_d,t_imax);
	AC_PID pid_yaw      (yPid.P,yPid.I,yPid.D,yPid.Imax);

    int gyrErrScale = 150;

    int timestamp;

    static LowPassFilter2p filt_x(update_hz,cutout);
	static LowPassFilter2p filt_y(update_hz,cutout);
	static LowPassFilter2p filt_z(update_hz,cutout);

	float offset_acc[3]  = {-.105,.800,1.0};
	Vector3f acc_offset(-.18, .66, 1.1);
	float scale_acc[3]   = {1,1,1};
	float offset_gyr[3]  = {-.001,-.0001,.0002};
	float scale_gyr[3]   = {1,1,1};

	int CNTRL_RANGE = 1000;
};

#endif
