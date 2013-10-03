#ifndef ros_Float64_h
#define ros_Float64_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class Float64 : public ros::Msg
  {
    public:
      float data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * val_data = (long *) &(this->data);
      long exp_data = (((*val_data)>>23)&255);
      if(exp_data != 0)
        exp_data += 1023-127;
      long sig_data = *val_data;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_data<<5) & 0xff;
      *(outbuffer + offset++) = (sig_data>>3) & 0xff;
      *(outbuffer + offset++) = (sig_data>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_data<<4) & 0xF0) | ((sig_data>>19)&0x0F);
      *(outbuffer + offset++) = (exp_data>>4) & 0x7F;
      if(this->data < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned long * val_data = (unsigned long*) &(this->data);
      offset += 3;
      *val_data = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_data |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_data |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_data |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_data = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_data |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_data !=0)
        *val_data |= ((exp_data)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->data = -this->data;
     return offset;
    }

    const char * getType(){ return "std_msgs/Float64"; };

  };

}
#endif