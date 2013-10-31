// DCM Vars
uint8_t sensors[6] = {1,2,0,4,5,6};  // For ArduPilot Mega Sensor Shield Hardware

int SENSORSIGN[]={1,-1,-1,-1,1,1,-1,-1,-1};

float DCM_Matrix[3][3]= { {1,0,0},{0,1,0},{0,0,1} }; 
float Update_Matrix[3][3]={ {0,1,2},{3,4,5},{6,7,8} };
float Temporary_Matrix[3][3]={ {0,0,0},{0,0,0},{0,0,0} };

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0}; //Store the gyros rotation rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator


float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
  
// Read all the ADC channels
void readADC(void)
{
  for ( int i = 0 ; i < 6 ; i++ ) {
    ADC_Ch[i] = APM_ADC.Ch(sensors[i]);
  }
}

// Returns an analog value with the offset
int readADCCorrected(int ch)
{
  if (SENSORSIGN[ch]<0)
    return (ADC_Offset[ch]-ADC_Ch[ch]);
  else
    return (ADC_Ch[ch]-ADC_Offset[ch]);
}

/* ******* DCM IMU functions ********************* */
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -VectorDotProduct(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  VectorScale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  VectorScale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  VectorAdd(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  VectorAdd(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  VectorCrossProduct(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - VectorDotProduct(&temporary[0][0],&temporary[0][0])); //eq.21
  VectorScale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
 
  renorm= .5 *(3 - VectorDotProduct(&temporary[1][0],&temporary[1][0])); //eq.21
  VectorScale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - VectorDotProduct(&temporary[2][0],&temporary[2][0])); //eq.21
  VectorScale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
  //Compensation the Roll, Pitch and Yaw drift. 
  float errorCourse;
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;

  // Calculate the magnitude of the accelerometer vector
  //Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  //Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // Accel_weight = constrain(1 - 4*abs(1 - Accel_magnitude),0,1);
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  //Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
  Accel_weight = -1.0;

  VectorCrossProduct(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  VectorScale(&Omega_P[0],&errorRollPitch[0],KpDCM_rollpitch*Accel_weight);
  
  VectorScale(&Scaled_Omega_I[0],&errorRollPitch[0],KiDCM_rollpitch*Accel_weight);
  VectorAdd(Omega_I,Omega_I,Scaled_Omega_I);
  
  // We make the gyro YAW drift correction based on compass magnetic heading 
  if ( MAGNETOMETER == 1) {
    errorCourse= (DCM_Matrix[0][0]*APM_Compass.Heading_Y) - (DCM_Matrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
    VectorScale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
 
    VectorScale(&Scaled_Omega_P[0],&errorYaw[0],KpDCM_yaw);
    VectorAdd(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

    VectorScale(&Scaled_Omega_I[0],&errorYaw[0],KiDCM_yaw);
    VectorAdd(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  }

}
/**************************************************/
void Accel_adjust(void)
{
  //Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
  //Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
}
/**************************************************/

void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(readADCCorrected(0)); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(readADCCorrected(1)); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(readADCCorrected(2)); //gyro Z yaw
  
  Accel_Vector[0]=readADCCorrected(3); // acc x
  Accel_Vector[1]=readADCCorrected(4); // acc y
  Accel_Vector[2]=readADCCorrected(5); // acc z
  
  // Low pass filter on accelerometer data (to filter vibrations)
  //Accel_Vector[0]=Accel_Vector[0]*0.5 + (float)read_adc(3)*0.5; // acc x
  //Accel_Vector[1]=Accel_Vector[1]*0.5 + (float)read_adc(4)*0.5; // acc y
  //Accel_Vector[2]=Accel_Vector[2]*0.5 + (float)read_adc(5)*0.5; // acc z
  
  VectorAdd(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);//adding integrator
  VectorAdd(&Omega_Vector[0], &Omega[0], &Omega_P[0]);//adding proportional
  
  //Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter
  
 //#if OUTPUTMODE==1 // corrected mode
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-loopDt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=loopDt*Omega_Vector[1];//y
  Update_Matrix[1][0]=loopDt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-loopDt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-loopDt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=loopDt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
  /*#endif
  #if OUTPUTMODE==0 // uncorrected data of the gyros (with drift)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-loopDt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=loopDt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=loopDt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-loopDt*Gyro_Vector[0];
  Update_Matrix[2][0]=-loopDt*Gyro_Vector[1];
  Update_Matrix[2][1]=loopDt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
  #endif*/

  MatrixMultiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++)  //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
    pitch = asin(-DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]) + ToRad(0.004129118*timeStep);
}

