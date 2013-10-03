#ifndef ros_PoseWithCovariance_h
#define ros_PoseWithCovariance_h

#include "WProgram.h"
#include "ros.h"
#include "geometry_msgs/Pose.h"

namespace geometry_msgs
{

  class PoseWithCovariance : public ros::Msg
  {
    public:
      geometry_msgs::Pose pose;
      float covariance[36];

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      unsigned char * covariance_val = (unsigned char *) this->covariance;
      for( unsigned char i = 0; i < 36; i++){
      long * val_covariancei = (long *) &(this->covariance[i]);
      long exp_covariancei = (((*val_covariancei)>>23)&255);
      if(exp_covariancei != 0)
        exp_covariancei += 1023-127;
      long sig_covariancei = *val_covariancei;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_covariancei<<5) & 0xff;
      *(outbuffer + offset++) = (sig_covariancei>>3) & 0xff;
      *(outbuffer + offset++) = (sig_covariancei>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_covariancei<<4) & 0xF0) | ((sig_covariancei>>19)&0x0F);
      *(outbuffer + offset++) = (exp_covariancei>>4) & 0x7F;
      if(this->covariance[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      unsigned char * covariance_val = (unsigned char *) this->covariance;
      for( unsigned char i = 0; i < 36; i++){
      unsigned long * val_covariancei = (unsigned long*) &(this->covariance[i]);
      offset += 3;
      *val_covariancei = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_covariancei |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_covariancei = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_covariancei |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_covariancei !=0)
        *val_covariancei |= ((exp_covariancei)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->covariance[i] = -this->covariance[i];
      }
     return offset;
    }

    const char * getType(){ return "geometry_msgs/PoseWithCovariance"; };

  };

}
#endif