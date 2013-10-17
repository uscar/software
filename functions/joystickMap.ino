#include <math.h>
#include <cmath>
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
void setup(){
  Serial.begin(57600);
  Serial.println(getSensitivity(900));
}
void loop(){
  
}
