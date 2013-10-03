#include <Wire.h>
#include <APM_RC.h>
#include <APM_ADC.h>
#include <APM_Compass.h>
#include <APM_BMP085.h>
#include <GPS_NMEA.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include "ART_Quadrotor.h"

#define USE_SERIAL
#define USE_ROS

void setup() {
  #ifdef USE_SERIAL
  Serial.begin(BAUDRATE);
  #endif
  
  Wire.begin();

  pinMode(LEDYELLOW,OUTPUT);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDGREEN,OUTPUT);
  pinMode(SWITCH1,INPUT);
  pinMode(SWITCH2,INPUT);  
  
  //for sonar pwm
  pinMode(15, INPUT);  

  APM_RC.Init();
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);
  /*APM_RC.OutputCh(0,2000);
   APM_RC.OutputCh(1,2000);
   APM_RC.OutputCh(2,2000);
   APM_RC.OutputCh(3,2000);*/
  delay(1000);  
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);  

  APM_ADC.Init();
  APM_BMP085.Init();
  //APM_Compass.Init();
  //GPS.Init();
  delay(100);  

  // Calibrate Gyros
  digitalWrite(LEDRED,HIGH);

  float aux_float[3];

  ADC_Offset[0] = gyroOffset[0];
  ADC_Offset[1] = gyroOffset[1];
  ADC_Offset[2] = gyroOffset[2];  
  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  readADC();
  delay(20);

  long averageGyro[3] = {
    0,0,0              };

  for ( int i = 0 ; i < 100 ; i++ ) {
    readADC();
    for ( int j = 0 ; j < 3 ; j++ ) {
      averageGyro[j] += ADC_Ch[j];
    }
    delay(30);
  }
  for ( int i = 0 ; i < 3 ; i++ ) {
    ADC_Offset[i] = averageGyro[i]/100;    
  }

  digitalWrite(LEDRED,LOW);

  calibrateLevel();
  
  // Init pressure altitude
  for ( unsigned int i = 0 ; i < 1000 ; i++ ) {
    readPressureAltitude();
    delay(0);
  }
  groundPressureAltitude = pressureAltitude;

  // Init ROS
  #ifdef USE_ROS
  nh.initNode();
  nh.advertise(rotation);
  nh.advertise(altitude);
  nh.subscribe(command_motors);
  nh.subscribe(command_altitude);
  nh.subscribe(command_waypoint);
  nh.subscribe(command_robot);
  #endif

  timer = millis();
  compassReadTimer = millis();  
  telemetryTimer = millis();
  otherTimer = millis();
}


// PID Tuning Packet
union PIDPacket
{
  struct
  {
    float roll[3];
    float pitch[3];
    float yaw[3];
    float altitude[3];
  };
  char data[48];
};


void loop() {

  // Sonar read  
  if ( millis() - sonarTimer > 100 ) {
    sonarTimer = millis();

    sonarAltitude = (pulseIn(15,HIGH)/ 58);
    /*
    if ( currentSonarData >= 8 ) {
      currentSonarData = 0;
    }

    sonarData[currentSonarData] = (pulseIn(15,HIGH)/ 58) ;
    currentSonarData++;

    int smallest = 2000;
    int smallest2 = 0;

    for ( unsigned int i = 0 ; i < 4 ; i++ ) {
      for ( unsigned int j = 0 ; j < 8 ; j++ ) {
        if ( sonarData[j] < smallest && sonarData[j] >= smallest2 ) {
          smallest = sonarData[j];
        }
      }
      smallest2 = smallest;
      smallest = 2000;
    }

    sonarAltitude = float(smallest2); //ft 
    smallest2 = 0;
    
    Serial.println(sonarAltitude); */
  }
  
/*  if ( abs((pressureAltitude-groundPressureAltitude)-sonarAltitude) > 1.0 ) { // 1.0 ft disagreement
    actualAltitude = pressureAltitude;
  } else {*/
  if (abs(actualAltitude - sonarAltitude) < 60) {
    actualAltitude = sonarAltitude;
  }
  //}

  if ( millis() - timer > 10 ) { // timer at 100 Hz

    loopDt = (millis()-timer)/1000.0;
    timer = millis();
    
    timeStep++;
    getMeasurements(); 
    
    // Read RC receiver
    for ( int i = 0 ; i < 4 ; i++ ) {
      RCInput[i] = radioFilter(APM_RC.InputCh(i),RCInput[i]);
    }

    for ( int i = 4 ; i < 8 ; i++ ) {
      RCInput[i] = APM_RC.InputCh(i);
    }

    pilotRollOld = pilotRoll;
    pilotRoll = 0.15*(RCInput[0]-MIDCHANNEL);
    pilotPitchOld = pilotPitch;
    pilotPitch = -0.15*(RCInput[1]-MIDCHANNEL);
    pilotThrottle = RCInput[2];
    pilotYawOld = pilotYaw;
    pilotYaw = 1.0*(RCInput[3]-MIDCHANNEL);

    PIDControl();

    if ( pilotThrottle < 1200 ) {
      controlYaw = 0;
      rollI = 0;
      pitchI = 0;
      if ( RCInput[3] > 1800 && RCInput[4] > 1500 ) {
        motorsArmed = 1;
      }
      if ( RCInput[3] < 1200 ) {
        motorsArmed = 0;
        isLanding = false;
      }
    }

    /* DO NOT DELETE: THIS IS FOR COMPUTER CONTROLLED ALTITUDE HOLD
     if ( RCInput[4] < 1500 && !isManualControl ) { // Computer controlled mode
     digitalWrite(LEDYELLOW,HIGH);            
     if ( holdingAltitude == false ) {
     altitudeI = 0;
     throttle = 1100;
     }       
     if ( isLanding == true ) {
     desiredAltitude = constrain(landingAltitude - ((millis() - landingTime)/75),0,200);
     if (sonarAltitude < 18) {
     isLanding = false;
     holdingAltitude = false;
     motorsArmed = 0;
     }        
     }
     } 
     else {
     isManualControl = true;
     controlAltitude = 0; 
     altitudeI = 0;
     throttle = pilotThrottle;
     digitalWrite(LEDYELLOW,LOW);      
     }
     */

    if ( RCInput[4] < 1500 ) {
      if ( holdingAltitude == false ) {
        holdingAltitude = true;        
        desiredAltitude = actualAltitude;
        desiredYpose = robotYpose;
        desiredThetapose = robotThetapose;
        desiredXpose = robotXpose;
        altitudeThrottle = pilotThrottle;
        controlAltitude = 0;
        controlYpose = 0;
        controlThetapose = 0;
        controlXpose = 0;
        digitalWrite(LEDYELLOW,HIGH);        
      } 
      if ( pilotThrottle < 1200 ) {
        motorsArmed = 0; // Can't rearm until altitude hold is turned off.
      }
      throttle = altitudeThrottle;      
    } 
    else {
      holdingAltitude = false;        
      controlAltitude = 0;
      altitudeI = 0;
      controlYpose = 0;
      YposeI = 0;
      controlThetapose = 0;
      ThetaposeI = 0;
      controlXpose = 0;
      XposeI = 0;
      throttle = pilotThrottle;
      digitalWrite(LEDYELLOW,LOW);
    }
    
    
    if ( motorsArmed == 1 ) {
      digitalWrite(LEDRED,HIGH);      
      motor[0] = constrain(throttle+controlRoll+controlPitch-controlYaw+controlAltitude+controlXpose+controlYpose-controlThetapose,1100,2000);
      motor[1] = constrain(throttle+controlRoll-controlPitch+controlYaw+controlAltitude+controlXpose-controlYpose+controlThetapose,1100,2000);
      motor[2] = constrain(throttle-controlRoll-controlPitch-controlYaw+controlAltitude-controlXpose-controlYpose-controlThetapose,1100,2000);
      motor[3] = constrain(throttle-controlRoll+controlPitch+controlYaw+controlAltitude-controlXpose+controlYpose+controlThetapose,1100,2000);
      APM_RC.OutputCh(0,motor[0]);
      APM_RC.OutputCh(1,motor[1]);
      APM_RC.OutputCh(2,motor[2]);
      APM_RC.OutputCh(3,motor[3]);
    } 
    else {
      digitalWrite(LEDRED,LOW);      
      APM_RC.OutputCh(0,1100);
      APM_RC.OutputCh(1,1100);
      APM_RC.OutputCh(2,1100);
      APM_RC.OutputCh(3,1100);
    }

  }

  if ( millis() - telemetryTimer > 200 ) {
    telemetryTimer = millis();
    //sendTestData();
  }

  /* ---- PID online tuning */
  if ( millis() - otherTimer > 20)
  {
    otherTimer = millis();
    //processPIDConstants();
    processPicoITXSerial();

  } 	/* ---- */
}

void PIDControl() {
  rollError = constrain(pilotRoll - ToDeg(roll),-60,60);

  rollI += rollError*loopDt;
  rollI = constrain(rollI,-20,20);

  rollD = ToDeg(-Omega[0]);

  controlRoll = Kproll*rollError + Kiroll*rollI + Kdroll*rollD;

  ///////////////////////////////////////

  pitchError = constrain(pilotPitch - ToDeg(pitch),-60,60);

  pitchI += pitchError*loopDt;
  pitchI = constrain(pitchI,-20,20);

  pitchD = ToDeg(-Omega[1]);

  controlPitch= Kppitch*pitchError + Kipitch*pitchI + Kdpitch*pitchD;

  /////////////////////////////////////////////

  yawError = constrain(pilotYaw - .5*readADCCorrected(2),-80,80);

  yawI += yawError*loopDt;
  yawI = constrain(yawI,-20,20);

  //yawD = readADCCorrected(2);

  controlYaw= Kpyaw*yawError + Kiyaw*yawI + Kdyaw*yawD;

  /////////////////////////////////////////////

  altitudeError = desiredAltitude-actualAltitude;

  altitudeI += altitudeError*loopDt;
  altitudeI = constrain(altitudeI,-1000,1000);

  altitudeD = (altitudeError - altitudeErrorOld)/loopDt;
  altitudeErrorOld = altitudeError;

  controlAltitude = Kpaltitude*altitudeError + Kialtitude*altitudeI + Kdaltitude*altitudeD;  

  /////////////////////////////////////////////
  
  XposeError = desiredXpose - robotXpose;
  
  XposeI += XposeError*loopDt;
  
  XposeD = (XposeError - XposeErrorOld)/loopDt;
  XposeErrorOld = XposeError;
  
  if (abs(XposeError) <= 1 && abs(ThetaposeError) <= 10) {
    controlXpose = KpXpose*XposeError + KiXpose*XposeI + KdXpose*XposeD;
  }
  
  /////////////////////////////////////////////
  
  YposeError = desiredYpose - robotYpose;
  
  YposeI += YposeError*loopDt;
  
  YposeD = (YposeError - YposeErrorOld)/loopDt;
  YposeErrorOld = YposeError;
  
  if (abs(ThetaposeError) <= 1.0) {
    controlYpose = KpYpose*YposeError + KiYpose*YposeI + KdYpose*YposeD;
  }
  
  /////////////////////////////////////////////
  
  ThetaposeError = desiredThetapose - robotThetapose;
  
  ThetaposeI += ThetaposeError*loopDt;
  
  controlThetapose = KpThetapose*ThetaposeError + KiThetapose*ThetaposeI;

}

// Maximum slope filter for radio inputs... (limit max differences between readings)
int radioFilter(int ch, int ch_old)
{
  int diff_ch_old;

  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old<0)
  {
    if (diff_ch_old<-40)
      return(ch_old-40);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old>40)    
      return(ch_old+40);
  }
  return(ch);
}

void getMeasurements() {
  readADC();
  if (MAGNETOMETER == 1) {
    if (millis() - compassReadTimer >= 100)  // Read compass data at 10Hz...
    {
      compassReadTimer = millis();
      APM_Compass.Read();     // Read magnetometer
      APM_Compass.Calculate(roll,pitch);  // Calculate heading
    }
  }
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();

  //  sonarAltitude = sonarAltitude*.9 + constrain(analogRead(A6),15,500)*.1;
}

void calibrateLevel() {
  digitalWrite(LEDRED,HIGH);

  // Calibrate accelOffset
  int average_x_offset = 0;
  int average_y_offset = 0;

  for ( int i = 0 ; i < 10 ; i++ ) {
    readADC();
    average_x_offset += ADC_Ch[3];
    average_y_offset += ADC_Ch[4];
    delay(20);
  }

  accelOffset[0] = average_x_offset/10;
  accelOffset[1] = average_y_offset/10;

  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  digitalWrite(LEDRED,LOW);
}

void readPressureAltitude() {
  // Barometer read
  APM_BMP085.Read();
  float tempPressureAltitude;
  tempPressureAltitude = float(APM_BMP085.Press)/(101325.0); //101325.0;
  tempPressureAltitude = pow(tempPressureAltitude,0.190295);
  if ( pressureAltitude == 0 ) {
    pressureAltitude = (1.0 - tempPressureAltitude) * 4433000 / 2.54 / 12; // ft
  } 
  else {
    pressureAltitude = pressureAltitude * 0.94 + ((1.0 - tempPressureAltitude) * 4433000 / 2.54 / 12)*0.06; // ft
  }
  //Serial.println(pressureAltitude - groundPressureAltitude);
}  






