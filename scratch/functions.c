#ifndef FUNCTIONS
#define FUNCTIONS

#include <math.h>
#include "structures.h"
#include "constants.h"
#include "WProgram.h"
#include <APM_ADC.h>
#include <APM_RC.h>

#define xMin 1180
#define xMax 1918
#define yMin 200
#define yMax 700

float getSensitivity(float xInput){
  float x = xInput - (xMax + xMin) / 2;
  float xRange = (xMax - xMin);
  float yRange = (yMax - yMin);
  float yInput = 2 * (yRange / pow(xRange, 3))*pow(x, 3) + .5*yRange*x / xRange;
  return yInput + (yMax + yMin) / 2;
}

float getSlopeAt(float xInput){
  float x = xInput - (xMax + xMin) / 2;
  float xRange = (xMax - xMin);
  float yRange = (yMax - yMin);
  return (6 * yRange / pow(xRange, 3) * pow(x, 2) + .5*yRange / xRange);
}

norm3 convertToNormal(f3 input){
  float magnitude=sqrt((input.x)*(input.x)+(input.y)*(input.y)+(input.z)*(input.z));
  float newX=input.x/magnitude;
  float newY=input.y/magnitude;
  float newZ=input.z/magnitude;
  f3 newVector;
  newVector.x=newX;
  newVector.y=newY;
  newVector.z=newZ;
  norm3 newNorm;
  newNorm.v=newVector;
  newNorm.m=magnitude;
  return newNorm;
}

float mag(float x, float y, float z){
  return sqrt(x*x+y*y+z*z);
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
void printn(norm3 out){
  Serial.print(out.m);
  Serial.print("<");
  Serial.print(out.v.x);
  Serial.print(",");
  Serial.print(out.v.y);
  Serial.print(",");
  Serial.print(out.v.z);
  Serial.println(">");
}

norm3 readController(int controller[4]){

  int throttle = controller[2]-CONTROL_MIN;
  int c_x = controller[0]-CONTROL_0;
  int c_y = controller[1]-CONTROL_0;

  int z = CONTROL_0-CONTROL_MIN;

  float m = mag((float)c_x,(float)c_y,(float)z);

  

  norm3 out;

  out.v.x = (float)c_x/m;
  out.v.y = (float)c_y/m;
  out.v.z = (float)z/m;
  out.m = (float)throttle;

  return out;
}


#endif


