/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
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

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(&rc1, &rc2, &rc3, &rc4);
AP_MotorsQuad   motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsHexa	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsY6	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsOcta	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsOctaQuad	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsHeli	motors(&rc1, &rc2, &rc3, &rc4);


// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");

    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_min_throttle(150);
    motors.set_mid_throttle(1000);
    motors.slow_start(false);
    motors.Init();      // initialise motors
    rc3.radio_min = 1150;
    rc3.radio_max = 1900;
    rc3.set_range(1150,1900);
    hal.console->printf("%i,%i",(int)rc3.radio_min,(int)rc3.radio_max);

    motors.enable();
    motors.output_min();

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int value;

    // display help
    hal.console->println("Press 't' to test motors.  Be careful they will spin!");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if( value == 't' || value == 'T' ) {
        hal.console->println("testing motors...");
        motors.armed(true);
        motors.output_test();
        motors.armed(false);
        hal.console->println("finished test.");
    }
    if( value == 'n'){
       hal.console->println("testing nam");
       motors.armed(true);
      for(int i = 0;i<100;i++){
         rc3.set_pwm(1100+8*i);
         hal.console->printf("\n%i %i",rc3.control_in,rc3.radio_in);
         hal.console->printf("\n%i - ",i*8+1100);
//       motors.output_min();
         motors.output();
//         motors.throttle_pass_through();
         print_motor_output();
         hal.scheduler->delay(50);
       }
       motors.output_min();
       motors.armed(false);
       hal.console->println("done"); 
    }
    if(value == 'b'){
      for(int i = 1150;i<2000;i+=10){
          hal.rcout->write(CH_1,i);
          hal.rcout->write(CH_3,i);
          hal.console->printf("outputting: %i\n",i);
          hal.scheduler->delay(20);
      }
      hal.scheduler->delay(500);
      hal.rcout->write(CH_1,0);
      hal.rcout->write(CH_3,0);
    }
}

// print motor output
void print_motor_output()
{
    int8_t i;
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if( motors.motor_enabled[i] ) {
            hal.console->printf("\t%d %d",i,motors.motor_out[i]);
        }
    }
}

AP_HAL_MAIN();
