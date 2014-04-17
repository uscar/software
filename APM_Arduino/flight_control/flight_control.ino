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

Flight_Control::Flight_Control(){

	motors = AP_MotorsQuad(&m_roll,&m_pitch,&m_throttle,&m_yaw);

	armed = false;
    ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_100HZ);

    // HAL will start serial port at 115200.
    hal.console->println_P(PSTR("Starting!"));

    //defines the mapping system from RC command to motor effect (defined in quad)
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);//motors.set_frame_orientation(AP_MOTORS_H_FRAME);
    //motors.min_throttle is in terms of servo values not output values.

    //setup rc
    setup_m_rc();

    //setup motors
    motors.Init();
    motors.set_update_rate(500);
    motors.enable();

    //setup timing
    t0 = timestamp = hal.scheduler->micros();

    //kill the barometer
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40,1);

    ///not the right place but it is a nice thought    
    old_cntrl_up = cntrl_up;
    old_cntrl_yaw = cntrl_yaw;


	pid = new AC_PID (h_p, h_i, h_d, h_imax);
}

void Flight_Control::execute(Vector3f up, float throttle, float yaw){

}