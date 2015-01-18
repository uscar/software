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

RC_Channel pitch(2), roll(3), throttle(1), yaw(4);

AP_MotorsQuad   motors(&pitch, &roll, &throttle, &yaw);
bool armed = false;
int PULSE_CHANGE = 1;
int motor_pulse = 0;

void print_motor_output(void) {
 int8_t i;
  for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
      if( motors.motor_enabled[i] ) {
          hal.console->printf("\n%d : %d", i, motors.motor_out[i]);
      }
  }
  hal.console->println(); 
}

void setup() {
  hal.console->println("Motor Test");
  pitch.radio_min = roll.radio_min = throttle.radio_min = yaw.radio_min = 1000;
  pitch.radio_max = roll.radio_max = throttle.radio_max = yaw.radio_max = 2000;
  pitch.set_range(1000,2000);
  roll.set_range(1000,2000);
  throttle.set_range(1000,2000);
  yaw.set_range(1000,2000);

  throttle.servo_out = 1;
  throttle.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  // motor initialisation
  motors.set_update_rate(490);
  motors.set_frame_orientation(AP_MOTORS_X_FRAME);
//    motors.set_min_throttle(1000);
//    motors.set_mid_throttle(1500);


  hal.console->printf("setup : %i\n",(int)motors.setup_throttle_curve());
  

  
  motors.Init();      // initialise motors

  hal.console->printf("%i,%i",(int)throttle.radio_min,(int)throttle.radio_max);

  motors.enable();
  motors.output_min();
  pitch.set_pwm(1000);
  roll.set_pwm(1000);
  yaw.set_pwm(1000);
  hal.scheduler->delay(1000);
}

void loop() {
  motors.armed(armed);
  if(hal.console->available()) {
    char val = hal.console->read();
    if(val == 's' || val == 'S') {
      armed = !armed;
      motors.armed(armed);
    }
    else if(val == 'u') {
      motor_pulse += PULSE_CHANGE;
    }
    else if(val == 'd') {
      motor_pulse -= PULSE_CHANGE;
    }
    else if(val == 'q') {
      motors.armed(false);
      motor_pulse = 0;
      pitch.servo_out = motor_pulse;
      roll.servo_out = motor_pulse;
      throttle.servo_out = motor_pulse;
      yaw.servo_out = motor_pulse;
      exit(1); 
    }
  }
  if(armed) {
    motors.output();
  }
  pitch.servo_out = motor_pulse;
  roll.servo_out = motor_pulse;
  throttle.servo_out = motor_pulse;
  yaw.servo_out = motor_pulse;
  hal.console->printf("armed: %d\n", armed);
  hal.console->printf("motor_pulse: %d\n", motor_pulse);
  print_motor_output();
  hal.scheduler->delay(100);
}

AP_HAL_MAIN();
