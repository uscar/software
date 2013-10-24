#include <APM_ADC.h>
#include <APM_RC.h>
#include "constants.h"
#include "functions.c"

int imu_cache[8];
int imu_read[8];

int rc_cache[8];
int rc_read[8];

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
  if(read_imu(imu_cache,imu_read))
    print8(imu_read);
  if(read_rc(rc_cache,rc_read))
    print8(rc_read);

  int throttle = rc_read[2]-CONTROL_MIN;
  int out = throttle+MOTOR_MIN;
  for(int i = 0;i<4;i++){
    APM_RC.OutputCh(i,out);
  }
  delay(10);
}

boolean read_imu(int cache[8],int out[8]){
  boolean ret = false;
  for(int i = 0;i<8;i++){
    out[i] = APM_ADC.Ch(i)-IMU_OFFSETS[i];
    if(abs(out[i] - cache[i])>=3){
      cache[i] = out[i];
      ret = true;
    }
  }
  return ret;
}

boolean read_rc(int cache[8],int out[8]){
  boolean ret = false;
  for(int i = 0;i<4;i++){
    out[i] = APM_RC.InputCh(i);
    if(abs(out[i]-cache[i])>=2){
      cache[i] = out[i];
      ret = true; 
    }
  } 
  return ret;
}

void print8(int out[8]){
  for(int i = 0;i<8;i++){
    Serial.print(out[i]);
    Serial.print(" ");
  }
  Serial.println();
}


