#include <APM_ADC.h>
#include <APM_RC.h>
#include "constants.h"

int imu_cache[8];
int imu_read[8];

//this will be called once (and only once) when it starts up
void setup(){
  APM_ADC.Init();
  APM_RC.Init();

  Serial.begin(57600);
  Serial.println("this will print out");


  //set the output channels to off for safety
  for(int i = 0;i<4;i++){
    APM_RC.OutputCh(i,MOTOR_OFF);
  }  

  delay(1000);
  //do it again for good measure.
  for(int i = 0;i<4;i++){
    APM_RC.OutputCh(i,MOTOR_OFF);
  }  

}

//this will be called over and over again
void loop(){
  boolean redraw = false;

  for(int i = 0;i<8;i++){
    imu_read[i] = APM_ADC.Ch(i)-IMU_OFFSETS[i];
    if(abs(imu_read[i] - imu_cache[i])>=3){
      imu_cache[i] = imu_read[i];
      redraw = true; 
    }
  }

  if(redraw){
    for(int i = 0;i<8;i++){
      Serial.print(imu_read[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  delay(10);
}



