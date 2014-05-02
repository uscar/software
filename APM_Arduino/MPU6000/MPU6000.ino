// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <GCS_MAVLink.h>
#include <AP_Notify.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor_MPU6000 ins;

void setup(void)
{
    hal.console->println("AP_InertialSensor startup...");

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);

    // display initial values
    display_offsets_and_scaling();
    hal.console->println("Complete. Reading:");
}

void loop(void)
{
    int16_t user_input;

    hal.console->println();
    hal.console->println_P(PSTR(
    "Menu:\r\n"
    "    c) calibrate accelerometers\r\n"
    "    d) display offsets and scaling\r\n"
    "    l) level (capture offsets from level)\r\n"
    "    t) test\r\n"
    "    r) reboot"));

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while( hal.console->available() ) {
        user_input = hal.console->read();

        if( user_input == 'c' || user_input == 'C' ) {
            run_calibration();
            display_offsets_and_scaling();
        }

        if( user_input == 'd' || user_input == 'D' ) {
            display_offsets_and_scaling();
        }

        if( user_input == 'l' || user_input == 'L' ) {
            run_level();
            display_offsets_and_scaling();
        }

        if( user_input == 't' || user_input == 'T' ) {
            run_test();
        }

        if( user_input == 'r' || user_input == 'R' ) {
			hal.scheduler->reboot(false);
        }
    }
}

void run_calibration()
{
    float roll_trim, pitch_trim;
    // clear off any other characters (like line feeds,etc)
    while( hal.console->available() ) {
        hal.console->read();
    }


#if !defined( __AVR_ATmega1280__ )
    AP_InertialSensor_UserInteractStream interact(hal.console);
    ins.calibrate_accel(&interact, roll_trim, pitch_trim);
#else
	hal.console->println_P(PSTR("calibrate_accel not available on 1280"));
#endif
}

void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf_P(
            PSTR("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_offsets.x,
                    accel_offsets.y,
                    accel_offsets.z);
    hal.console->printf_P(
            PSTR("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_scale.x,
                    accel_scale.y,
                    accel_scale.z);
    hal.console->printf_P(
            PSTR("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z);
}

void run_level()
{
    // clear off any input in the buffer
    while( hal.console->available() ) {
        hal.console->read();
    }

    // display message to user
    hal.console->print("Place APM on a level surface and press any key..\n");

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }
    while( hal.console->available() ) {
        hal.console->read();
    }

    // run accel level
    ins.init_accel();

    // display results
    display_offsets_and_scaling();
}


void run_test()
{
    Vector3f accel;
    Vector3f prev_accel;

    
    float lenA, Gtotal = 0, gravity = 0, mill=0, acc = 0,
          Vtotal = 0, time = 0, num = 0, V_last = 0, same = 0;
	uint8_t counter = 0;

    // flush any user input
    while( hal.console->available() ) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    float prev_time = hal.scheduler->millis();

    // loop as long as user does not press a key
    while( !hal.console->available() ) {
  
        // wait until we have a sample
        while (ins.sample_available() == false) /* noop */ ;

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        
        // calibrations
        accel.x = (1.001268*accel.x) - 0.00869187;
        accel.y = (1.004233*accel.y) + 0.02;
        accel.z = (0.9863417*accel.z) + 0.8110786;
        
        lenA = accel.length();
        
        /*
        Calibration variable
        avg magnitude of acceleration = g
        from start time to current time
        
        then take ( |a| - g) and integrate
        */
        
        if(counter++ % 10) {
         hal.console->printf_P(PSTR("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \t g:%4.2f \t ACCEL:%5.2f\t V:%4.2f\t time:%4.2f\n"), 
								  accel.x, accel.y, accel.z, lenA, gravity, acc, Vtotal, time);
        }
        mill = hal.scheduler->millis() - prev_time;
        if(mill) {
           if(lenA != 0)
           {
             time += mill;
             if(time < 10000)
             {
               Gtotal = lenA + Gtotal;
               ++num;
               gravity = Gtotal / num;
             }  
             
             // Deal with negative value
             if(accel.z > 0)
               lenA = -1*lenA;
             acc = lenA - gravity;
             
             if(acc > 0.5 || acc < -0.5) // evaluates velocity
               Vtotal = (acc*mill/1000) + Vtotal;
               
              if(Vtotal == V_last && Vtotal != 0)
              {
                ++same;
                if(same > 10)
                {
                  Vtotal = 0;
                  same = 0;
                }
              }
              V_last = Vtotal;
           }
           else
           {
             Gtotal = lenA;
           }
        prev_time = hal.scheduler->millis();
        }
    }

    // clear user input
    while( hal.console->available() ) {
        hal.console->read();
    }
}


AP_HAL_MAIN();
