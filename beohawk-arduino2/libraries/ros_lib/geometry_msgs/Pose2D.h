#ifndef ros_Pose2D_h
#define ros_Pose2D_h

#include "WProgram.h"
#include "ros.h"

namespace geometry_msgs
{

  class Pose2D : public ros::Msg
  {
    public:
      float x;
      float y;
      float theta;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * val_x = (long *) &(this->x);
      long exp_x = (((*val_x)>>23)&255);
      if(exp_x != 0)
        exp_x += 1023-127;
      long sig_x = *val_x;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_x<<5) & 0xff;
      *(outbuffer + offset++) = (sig_x>>3) & 0xff;
      *(outbuffer + offset++) = (sig_x>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_x<<4) & 0xF0) | ((sig_x>>19)&0x0F);
      *(outbuffer + offset++) = (exp_x>>4) & 0x7F;
      if(this->x < 0) *(outbuffer + offset -1) |= 0x80;
      long * val_y = (long *) &(this->y);
      long exp_y = (((*val_y)>>23)&255);
      if(exp_y != 0)
        exp_y += 1023-127;
      long sig_y = *val_y;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_y<<5) & 0xff;
      *(outbuffer + offset++) = (sig_y>>3) & 0xff;
      *(outbuffer + offset++) = (sig_y>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_y<<4) & 0xF0) | ((sig_y>>19)&0x0F);
      *(outbuffer + offset++) = (exp_y>>4) & 0x7F;
      if(this->y < 0) *(outbuffer + offset -1) |= 0x80;
      long * val_theta = (long *) &(this->theta);
      long exp_theta = (((*val_theta)>>23)&255);
      if(exp_theta != 0)
        exp_theta += 1023-127;
      long sig_theta = *val_theta;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_theta<<5) & 0xff;
      *(outbuffer + offset++) = (sig_theta>>3) & 0xff;
      *(outbuffer + offset++) = (sig_theta>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_theta<<4) & 0xF0) | ((sig_theta>>19)&0x0F);
      *(outbuffer + offset++) = (exp_theta>>4) & 0x7F;
      if(this->theta < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned long * val_x = (unsigned long*) &(this->x);
      offset += 3;
      *val_x = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_x |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_x |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_x |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_x = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_x |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_x !=0)
        *val_x |= ((exp_x)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->x = -this->x;
      unsigned long * val_y = (unsigned long*) &(this->y);
      offset += 3;
      *val_y = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_y |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_y |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_y |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_y = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_y |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_y !=0)
        *val_y |= ((exp_y)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->y = -this->y;
      unsigned long * val_theta = (unsigned long*) &(this->theta);
      offset += 3;
      *val_theta = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_theta |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_theta |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_theta |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_theta = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_theta |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_theta !=0)
        *val_theta |= ((exp_theta)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->theta = -this->theta;
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Pose2D"; };

  };

}
#endif