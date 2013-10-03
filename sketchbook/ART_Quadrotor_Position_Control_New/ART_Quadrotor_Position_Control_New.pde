#include <ArduinoHardware.h>
#include <Wire.h>
#include <APM_RC.h>
#include <APM_ADC.h>
#include <APM_Compass.h>
#include <APM_BMP085.h>
#include <EEPROM.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
//#include <GPS_NMEA.h>

#include "ART_Quadrotor.h"

// Initialize ROS Node
ros::NodeHandle nh;

void cb_position (const geometry_msgs::Point32& msg_position);

geometry_msgs::Point32 msg_pos;
geometry_msgs::PoseStamped msg_attitude;
ros::Publisher pub_attitude("/arduino/attitude", &msg_attitude);
ros::Subscriber<geometry_msgs::Point32> sub_position("/arduino/pos", &cb_position);
void cb_position (const geometry_msgs::Point32& msg_position) {
  laserXPosition = msg_position.x;
  laserYPosition = msg_position.y;
  laserYaw = msg_position.z;
  //Serial.print(laserXPosition);  Serial.print("     ");  Serial.print(laserYPosition);  Serial.print("     ");  Serial.println(laserYaw);
  nh.spinOnce();
}

//#define USE_SERIAL


void setup() {
#ifdef USE_SERIAL
  Serial.begin(BAUDRATE);
#endif

// Initialize ROS
  nh.initNode();
  nh.advertise(pub_attitude);
  nh.subscribe(sub_position);

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
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);  

  APM_ADC.Init();
  //APM_BMP085.Init();
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
    0,0,0  
  };

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

  /* Init pressure altitude
  for ( unsigned int i = 0 ; i < 1000 ; i++ ) {
    readPressureAltitude();
  }
  
  groundPressureAltitude = pressureAltitude;
*/
  timer1 = millis();
  compassReadTimer = millis();  
  telemetryTimer = millis();
  otherTimer = millis();
  rosTimer = millis();

}


int counter1 = 0;
  
void loop() {
  biasRoll = EEPROM.read(0) - 128;
  biasPitch = EEPROM.read(1) - 128;
  
  biasRoll *= maxAutoBias;
  biasPitch *= maxAutoBias;

  //Sonar Sensor Data
  if ( millis() - sonarTimer > 88 ) {
    loopDt_slow = (millis() - sonarTimer)/1000.0;
    //Serial.println(throttle);
    
    //sonarAltitude = (pulseIn(15,HIGH)/147.0);
    
    
    sonarAltitude = analogRead(7);
    sonarTimer = millis();
    
    if (sonarAltitude > 700)
    {
      sonarAltitude = 20;
    }
    
    if (abs(previousSonarAltitude - sonarAltitude) < 10)
    {
      usedAltitude = sonarAltitude;
    }
      
    previousSonarAltitude = sonarAltitude;   
    
    previousActualAltitude = actualAltitude;
    actualAltitude = 0.4*actualAltitude + (1-0.4)*usedAltitude;

  }  
  

  if ( millis() - timer1 > 10 ) { // timer at 100 Hz
    //Serial.println(millis() - timer1);
    loopDt = (millis()-timer1)/1000.0;
    timer1 = millis();
    timeStep++;
    
    
    getMeasurements();
    Serial.println(ToDeg(roll));
    pitch = -pitch; 
    yaw = -yaw;
    
    //Serial.print(pitch);    Serial.print("  ");    Serial.print(roll);    Serial.print("  ");    Serial.println(yaw);

    // Read RC receiver
    for ( int i = 0 ; i < 4 ; i++ ) {
      RCInput[i] = radioFilter(APM_RC.InputCh(i),RCInput[i]);
      //Serial.print(i); Serial.print (": ");Serial.print(RCInput[i]);Serial.print("    ");
    }
    //Serial.println();

    for ( int i = 4 ; i < 8 ; i++ ) {
      RCInput[i] = APM_RC.InputCh(i);
      
    }
    //Serial.println("");
    

    pilotRollOld = pilotRoll;
    pilotRoll = 0.175*(RCInput[0]-MIDCHANNEL) + trimRoll*trimPitchRollRate;
    pilotPitchOld = pilotPitch;
    pilotPitch = -0.175*(RCInput[1]-MIDCHANNEL) + trimPitch*trimPitchRollRate;
    pilotThrottle = RCInput[2];
    //Serial.println(pilotThrottle);
    pilotYawOld = pilotYaw;
    pilotYaw = 2.0*(RCInput[3]-1509);
   // Serial.print(pilotYaw); Serial.print("  "); Serial.println(trimYaw/10);
      //Throttle setting  

   if (RCInput[4] < 1550) //autoTrim will be on
   {
    
     autoTrim = true;
     trimRoll += loopDt*(pilotRoll - trimRoll*trimPitchRollRate);
     trimPitch += loopDt*(pilotPitch - trimPitch*trimPitchRollRate);
     trimYaw += loopDt*(pilotYaw);
     
      if ((millis() - EEPROMClearTimer) < 1000) 
     {
       trimRoll = 0;
       trimPitch = 0;
       trimYaw = 0;
       for (int i = 0; i <= 4; i++)
       {
          EEPROM.write(i, 128);
       }
     }
     //Serial.print(trimPitch); Serial.print("  "); Serial.println(pilotPitch);
   } else {
     if (autoTrim == true)
     {
       autoTrim = false;
       EEPROMClearTimer = millis();
       EEPROM.write(0,constrain((biasRoll + rollI)/maxAutoBias + 128,0,255));
       rollI = 0;
       EEPROM.write(1,constrain((biasPitch + pitchI)/maxAutoBias + 128,0,255));
       pitchI = 0;
       EEPROM.write(2,constrain((trimRoll/trimPitchRollRange+128),0,255));
       EEPROM.write(3,constrain((trimPitch/trimPitchRollRange+128),0,255));
       EEPROM.write(4,constrain((trimYaw/trimYawRange+128),0,255));
     } else {
       autoTrim = false;
     }
       trimRoll = trimPitchRollRange*(EEPROM.read(2) - 128);
       trimPitch = trimPitchRollRange*(EEPROM.read(3) - 128);
       trimYaw = trimYawRange*(EEPROM.read(4) - 128);
   }


   if (RCInput[5] < 1550) //Altitude hold and position hold will be on
   {
     if (altHold == false) {
       //holdingThrottle = 1515;
       //holdingAltitude = 60.0;
       //desiredAltitude = 60.0;
       holdingThrottle = pilotThrottle;
       holdingAltitude = actualAltitude;
       if (holdingAltitude > 200)
       {
         holdingAltitude = 20;
       }
       altHold = true;
       altHoldStart = millis();
     }
     throttle = holdingThrottle;
     desiredAltitude = (pilotThrottle - 1100)/7;
   
   if (positionHold == false)
      {
        positionHold = true;
        desiredYaw = laserYaw;
        desiredXPosition = laserXPosition;
        desiredYPosition = laserYPosition;
      }
      desiredPitch = -10*(1/(1+exp(-5*(desiredXPosition-laserXPosition)))-1/2);
      desiredRoll = 10*(1/(1+exp(-5*(desiredYPosition-laserYPosition)))-1/2);
      desiredYawChange = (desiredYaw - laserYaw)*10;
      
    } else {
      altHold = false;
      throttle = pilotThrottle;
      desiredAltitude = actualAltitude;
      positionHold = false;    
      desiredPitch = pilotPitch;
      desiredRoll = pilotRoll;
      desiredYawChange = pilotYaw;
    }  
    
    if (positionHold == true)
    {
      desiredPitch = -10*(1/(1+exp(-5*desiredXPosition))-1/2);
      desiredRoll = 10*(1/(1+exp(-5*desiredYPosition))-1/2);
    } else {
      desiredPitch = pilotPitch;
      desiredRoll = pilotRoll;
    }
     
   //desiredAltitude = constrain(map(pilotThrottle,1100,2000, 17,100),10,70);     

     
 
 
   PIDControl();
   
   //Arm Motors 
   if ( pilotThrottle < 1200 ) {
     
       if ( RCInput[3] > 1700 && RCInput[5] > 1550) {
           motorsArmed = 1;
       }
       if ( RCInput[3] < 1200 ) {
           motorsArmed = 0;
           isLanding = false;
       }
   }
   
   //Altitude Hold

   if (motorsArmed == 0) {
     altitudeI = 0; 
     yawI = 0;
     rollI = 0;
     pitchI = 0;
     desiredAltitude = 0;
   }
   if (altHold == false || (millis()-altHoldStart) < altHoldDelay)
     altitudeI = 0;

 // DO NOT DELETE: THIS IS FOR COMPUTER CONTROLLED ALTITUDE HOLD
  // Computer controlled mode
       digitalWrite(LEDYELLOW,HIGH); 
       //Serial.println("ALT HOLD");       
  
       
    
   
   isManualControl = true;
   
   digitalWrite(LEDYELLOW,LOW);  

   //controlAltitude = 0;
   digitalWrite(LEDYELLOW,HIGH); 
          
   //xSerial.println(actualAltitude);
   digitalWrite(LEDYELLOW,LOW);
   //Serial.print(desiredAltitude); Serial.print("  "); Serial.print(holdingAltitude); Serial.print("  "); Serial.print(actualAltitude); Serial.print("  ") ;Serial.print(altitudeI); Serial.print("   "); Serial.print(controlAltitude); Serial.print("  "); Serial.println(throttle);
   //Serial.print(controlRoll); Serial.print("   "); Serial.print(controlPitch); Serial.print("   ");    Serial.println(controlYaw);
   //Serial.print(actualAltitude);Serial.print("  "); Serial.print(desiredAltitude); Serial.print("  "); Serial.print(altitudeI); Serial.print("  "); Serial.println(controlAltitude);
   //Serial.print(pitchI); Serial.print("   "); Serial.print(rollI); Serial.print("   ");    Serial.println(yawI);
   if ( motorsArmed == 1 ) {
     digitalWrite(LEDRED,HIGH);    
     if (altHold == false || (millis()-altHoldStart) < altHoldDelay)
        controlAltitude = 0;
      
      
              
      motor[0] = constrain(throttle+controlRoll+controlPitch-controlYaw+(controlAltitude),1100,2000);
      motor[1] = constrain(throttle+controlRoll-controlPitch+controlYaw+(controlAltitude),1100,2000);
      motor[2] = constrain(throttle-controlRoll-controlPitch-controlYaw+(controlAltitude),1100,2000);
      motor[3] = constrain(throttle-controlRoll+controlPitch+controlYaw+(controlAltitude),1100,2000);
      
      //Serial.println(motor[0]); Serial.print("  =  "); Serial.print(controlPitch); Serial.print(" + "); Serial.print(controlRoll);Serial.print(" + "); Serial.print(controlYaw); Serial.print(" + "); Serial.print(controlAltitude); Serial.print("*"); Serial.println(flag);

      
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
  
  //Ros msg data
  if ( millis() - rosTimer > 100) { 
    msg_attitude.pose.orientation.x = ToDeg(roll);
    msg_attitude.pose.orientation.y = ToDeg(pitch);
    msg_attitude.pose.orientation.z = ToDeg(yaw);
    msg_attitude.header.stamp = nh.now();
    pub_attitude.publish(&msg_attitude);
    nh.spinOnce();
    rosTimer = millis();
  }
 
}

void PIDControl() {
  rollError = constrain(desiredRoll - ToDeg(roll),-60,60);

  rollI += rollError*loopDt;
  if (autoTrim == false)
  {
    rollI = constrain(rollI,-pitchRollIMax,pitchRollIMax);
  } else {
    rollI = constrain(rollI,-5*pitchRollIMax,5*pitchRollIMax);
  }

  rollD = ToDeg(-Omega[0]);

  controlRoll = Kproll*rollError + Kiroll*(rollI+biasRoll) + Kdroll*rollD;

  ///////////////////////////////////////

  pitchError = constrain(desiredPitch - ToDeg(pitch),-60,60);

  pitchI += pitchError*loopDt;
  if (autoTrim == false)
  {
    pitchI = constrain(pitchI,-pitchRollIMax,pitchRollIMax);
  } else {
    pitchI = constrain(pitchI,-5*pitchRollIMax,5*pitchRollIMax);    
  }

  pitchD = ToDeg(Omega[1]);

  controlPitch= Kppitch*pitchError + Kipitch*(pitchI+biasPitch) + Kdpitch*pitchD;
  //Serial.print(autoTrim); Serial.print("  ");Serial.print(ToDeg(pitch)); Serial.print("  ");  Serial.print(pilotPitch); Serial.print("  ");  Serial.print(pitchError); Serial.print("  ");  Serial.print(pitchI); Serial.print("  "); Serial.print(trimPitch); Serial.print("  "); Serial.println(biasPitch); 

  /////////////////////////////////////////////

  yawError = constrain(desiredYawChange + trimYaw*trimYawRate,-250,250) + constrain(.5*readADCCorrected(2),-80,80);
  //Serial.println(readADCCorrected(2));
  //Serial.println(pilotYaw);

  yawI += yawError*loopDt;
  yawI = constrain(yawI,-30,30);

  //yawD = readADCCorrected(2);

  controlYaw= Kpyaw*(yawError) + Kiyaw*yawI + Kdyaw*yawD;

  /////////////////////////////////////////////

  altitudeError = desiredAltitude-actualAltitude;
  
  
  altitudeI += altitudeError*loopDt;
  altitudeI = constrain(altitudeI,-altitudeIMax,altitudeIMax);

  altitudeD = (previousActualAltitude - actualAltitude)/loopDt_slow;
  altitudeErrorOld = altitudeError;
  
  //Serial.print(Kpaltitude*altitudeError); Serial.print("  "); Serial.print(Kialtitude*altitudeI); Serial.print("  "); Serial.print(Kdaltitude*altitudeD); Serial.print("  ") ;Serial.print(actualAltitude); Serial.print("   "); Serial.println(desiredAltitude);

  controlAltitude = Kpaltitude*altitudeError + Kialtitude*altitudeI + Kdaltitude*altitudeD;  

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
    pressureAltitude = (1.0 - tempPressureAltitude) * 4433000 / 2.54; // in
  } 
  else {
    pressureAltitude = pressureAltitude * 0.94 + ((1.0 - tempPressureAltitude) * 4433000 / 2.54)*0.06; // in
  }
  //Serial.println(pressureAltitude - groundPressureAltitude);
}  
