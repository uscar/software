#ifndef ros_NavSatFix_h
#define ros_NavSatFix_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatStatus.h"

namespace sensor_msgs
{

  class NavSatFix : public ros::Msg
  {
    public:
      std_msgs::Header header;
      sensor_msgs::NavSatStatus status;
      float latitude;
      float longitude;
      float altitude;
      float position_covariance[9];
      unsigned char position_covariance_type;
      enum { COVARIANCE_TYPE_UNKNOWN = 0 };
      enum { COVARIANCE_TYPE_APPROXIMATED = 1 };
      enum { COVARIANCE_TYPE_DIAGONAL_KNOWN = 2 };
      enum { COVARIANCE_TYPE_KNOWN = 3 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      long * val_latitude = (long *) &(this->latitude);
      long exp_latitude = (((*val_latitude)>>23)&255);
      if(exp_latitude != 0)
        exp_latitude += 1023-127;
      long sig_latitude = *val_latitude;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_latitude<<5) & 0xff;
      *(outbuffer + offset++) = (sig_latitude>>3) & 0xff;
      *(outbuffer + offset++) = (sig_latitude>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_latitude<<4) & 0xF0) | ((sig_latitude>>19)&0x0F);
      *(outbuffer + offset++) = (exp_latitude>>4) & 0x7F;
      if(this->latitude < 0) *(outbuffer + offset -1) |= 0x80;
      long * val_longitude = (long *) &(this->longitude);
      long exp_longitude = (((*val_longitude)>>23)&255);
      if(exp_longitude != 0)
        exp_longitude += 1023-127;
      long sig_longitude = *val_longitude;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_longitude<<5) & 0xff;
      *(outbuffer + offset++) = (sig_longitude>>3) & 0xff;
      *(outbuffer + offset++) = (sig_longitude>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_longitude<<4) & 0xF0) | ((sig_longitude>>19)&0x0F);
      *(outbuffer + offset++) = (exp_longitude>>4) & 0x7F;
      if(this->longitude < 0) *(outbuffer + offset -1) |= 0x80;
      long * val_altitude = (long *) &(this->altitude);
      long exp_altitude = (((*val_altitude)>>23)&255);
      if(exp_altitude != 0)
        exp_altitude += 1023-127;
      long sig_altitude = *val_altitude;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_altitude<<5) & 0xff;
      *(outbuffer + offset++) = (sig_altitude>>3) & 0xff;
      *(outbuffer + offset++) = (sig_altitude>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_altitude<<4) & 0xF0) | ((sig_altitude>>19)&0x0F);
      *(outbuffer + offset++) = (exp_altitude>>4) & 0x7F;
      if(this->altitude < 0) *(outbuffer + offset -1) |= 0x80;
      unsigned char * position_covariance_val = (unsigned char *) this->position_covariance;
      for( unsigned char i = 0; i < 9; i++){
      long * val_position_covariancei = (long *) &(this->position_covariance[i]);
      long exp_position_covariancei = (((*val_position_covariancei)>>23)&255);
      if(exp_position_covariancei != 0)
        exp_position_covariancei += 1023-127;
      long sig_position_covariancei = *val_position_covariancei;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_position_covariancei<<5) & 0xff;
      *(outbuffer + offset++) = (sig_position_covariancei>>3) & 0xff;
      *(outbuffer + offset++) = (sig_position_covariancei>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_position_covariancei<<4) & 0xF0) | ((sig_position_covariancei>>19)&0x0F);
      *(outbuffer + offset++) = (exp_position_covariancei>>4) & 0x7F;
      if(this->position_covariance[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      union {
        unsigned char real;
        unsigned char base;
      } u_position_covariance_type;
      u_position_covariance_type.real = this->position_covariance_type;
      *(outbuffer + offset + 0) = (u_position_covariance_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      unsigned long * val_latitude = (unsigned long*) &(this->latitude);
      offset += 3;
      *val_latitude = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_latitude |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_latitude |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_latitude |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_latitude = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_latitude |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_latitude !=0)
        *val_latitude |= ((exp_latitude)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->latitude = -this->latitude;
      unsigned long * val_longitude = (unsigned long*) &(this->longitude);
      offset += 3;
      *val_longitude = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_longitude |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_longitude |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_longitude |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_longitude = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_longitude |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_longitude !=0)
        *val_longitude |= ((exp_longitude)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->longitude = -this->longitude;
      unsigned long * val_altitude = (unsigned long*) &(this->altitude);
      offset += 3;
      *val_altitude = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_altitude |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_altitude |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_altitude |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_altitude = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_altitude |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_altitude !=0)
        *val_altitude |= ((exp_altitude)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->altitude = -this->altitude;
      unsigned char * position_covariance_val = (unsigned char *) this->position_covariance;
      for( unsigned char i = 0; i < 9; i++){
      unsigned long * val_position_covariancei = (unsigned long*) &(this->position_covariance[i]);
      offset += 3;
      *val_position_covariancei = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_position_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_position_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_position_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_position_covariancei = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_position_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_position_covariancei !=0)
        *val_position_covariancei |= ((exp_position_covariancei)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->position_covariance[i] = -this->position_covariance[i];
      }
      union {
        unsigned char real;
        unsigned char base;
      } u_position_covariance_type;
      u_position_covariance_type.base = 0;
      u_position_covariance_type.base |= ((typeof(u_position_covariance_type.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->position_covariance_type = u_position_covariance_type.real;
      offset += sizeof(this->position_covariance_type);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/NavSatFix"; };

  };

}
#endif