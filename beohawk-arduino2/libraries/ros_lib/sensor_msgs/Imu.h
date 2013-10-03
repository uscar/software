#ifndef ros_Imu_h
#define ros_Imu_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

namespace sensor_msgs
{

  class Imu : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Quaternion orientation;
      float orientation_covariance[9];
      geometry_msgs::Vector3 angular_velocity;
      float angular_velocity_covariance[9];
      geometry_msgs::Vector3 linear_acceleration;
      float linear_acceleration_covariance[9];

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      unsigned char * orientation_covariance_val = (unsigned char *) this->orientation_covariance;
      for( unsigned char i = 0; i < 9; i++){
      long * val_orientation_covariancei = (long *) &(this->orientation_covariance[i]);
      long exp_orientation_covariancei = (((*val_orientation_covariancei)>>23)&255);
      if(exp_orientation_covariancei != 0)
        exp_orientation_covariancei += 1023-127;
      long sig_orientation_covariancei = *val_orientation_covariancei;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_orientation_covariancei<<5) & 0xff;
      *(outbuffer + offset++) = (sig_orientation_covariancei>>3) & 0xff;
      *(outbuffer + offset++) = (sig_orientation_covariancei>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_orientation_covariancei<<4) & 0xF0) | ((sig_orientation_covariancei>>19)&0x0F);
      *(outbuffer + offset++) = (exp_orientation_covariancei>>4) & 0x7F;
      if(this->orientation_covariance[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      offset += this->angular_velocity.serialize(outbuffer + offset);
      unsigned char * angular_velocity_covariance_val = (unsigned char *) this->angular_velocity_covariance;
      for( unsigned char i = 0; i < 9; i++){
      long * val_angular_velocity_covariancei = (long *) &(this->angular_velocity_covariance[i]);
      long exp_angular_velocity_covariancei = (((*val_angular_velocity_covariancei)>>23)&255);
      if(exp_angular_velocity_covariancei != 0)
        exp_angular_velocity_covariancei += 1023-127;
      long sig_angular_velocity_covariancei = *val_angular_velocity_covariancei;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_angular_velocity_covariancei<<5) & 0xff;
      *(outbuffer + offset++) = (sig_angular_velocity_covariancei>>3) & 0xff;
      *(outbuffer + offset++) = (sig_angular_velocity_covariancei>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_angular_velocity_covariancei<<4) & 0xF0) | ((sig_angular_velocity_covariancei>>19)&0x0F);
      *(outbuffer + offset++) = (exp_angular_velocity_covariancei>>4) & 0x7F;
      if(this->angular_velocity_covariance[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      unsigned char * linear_acceleration_covariance_val = (unsigned char *) this->linear_acceleration_covariance;
      for( unsigned char i = 0; i < 9; i++){
      long * val_linear_acceleration_covariancei = (long *) &(this->linear_acceleration_covariance[i]);
      long exp_linear_acceleration_covariancei = (((*val_linear_acceleration_covariancei)>>23)&255);
      if(exp_linear_acceleration_covariancei != 0)
        exp_linear_acceleration_covariancei += 1023-127;
      long sig_linear_acceleration_covariancei = *val_linear_acceleration_covariancei;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_linear_acceleration_covariancei<<5) & 0xff;
      *(outbuffer + offset++) = (sig_linear_acceleration_covariancei>>3) & 0xff;
      *(outbuffer + offset++) = (sig_linear_acceleration_covariancei>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_linear_acceleration_covariancei<<4) & 0xF0) | ((sig_linear_acceleration_covariancei>>19)&0x0F);
      *(outbuffer + offset++) = (exp_linear_acceleration_covariancei>>4) & 0x7F;
      if(this->linear_acceleration_covariance[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      unsigned char * orientation_covariance_val = (unsigned char *) this->orientation_covariance;
      for( unsigned char i = 0; i < 9; i++){
      unsigned long * val_orientation_covariancei = (unsigned long*) &(this->orientation_covariance[i]);
      offset += 3;
      *val_orientation_covariancei = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_orientation_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_orientation_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_orientation_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_orientation_covariancei = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_orientation_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_orientation_covariancei !=0)
        *val_orientation_covariancei |= ((exp_orientation_covariancei)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->orientation_covariance[i] = -this->orientation_covariance[i];
      }
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      unsigned char * angular_velocity_covariance_val = (unsigned char *) this->angular_velocity_covariance;
      for( unsigned char i = 0; i < 9; i++){
      unsigned long * val_angular_velocity_covariancei = (unsigned long*) &(this->angular_velocity_covariance[i]);
      offset += 3;
      *val_angular_velocity_covariancei = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_angular_velocity_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_angular_velocity_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_angular_velocity_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_angular_velocity_covariancei = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_angular_velocity_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_angular_velocity_covariancei !=0)
        *val_angular_velocity_covariancei |= ((exp_angular_velocity_covariancei)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->angular_velocity_covariance[i] = -this->angular_velocity_covariance[i];
      }
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      unsigned char * linear_acceleration_covariance_val = (unsigned char *) this->linear_acceleration_covariance;
      for( unsigned char i = 0; i < 9; i++){
      unsigned long * val_linear_acceleration_covariancei = (unsigned long*) &(this->linear_acceleration_covariance[i]);
      offset += 3;
      *val_linear_acceleration_covariancei = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_linear_acceleration_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_linear_acceleration_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_linear_acceleration_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_linear_acceleration_covariancei = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_linear_acceleration_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_linear_acceleration_covariancei !=0)
        *val_linear_acceleration_covariancei |= ((exp_linear_acceleration_covariancei)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->linear_acceleration_covariance[i] = -this->linear_acceleration_covariance[i];
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/Imu"; };

  };

}
#endif