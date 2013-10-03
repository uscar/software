#ifndef ros_CameraInfo_h
#define ros_CameraInfo_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace sensor_msgs
{

  class CameraInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned long height;
      unsigned long width;
      unsigned char * distortion_model;
      unsigned char D_length;
      float st_D;
      float * D;
      float K[9];
      float R[9];
      float P[12];
      unsigned long binning_x;
      unsigned long binning_y;
      sensor_msgs::RegionOfInterest roi;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        unsigned long real;
        unsigned long base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      long * length_distortion_model = (long *)(outbuffer + offset);
      *length_distortion_model = strlen( (const char*) this->distortion_model);
      offset += 4;
      memcpy(outbuffer + offset, this->distortion_model, *length_distortion_model);
      offset += *length_distortion_model;
      *(outbuffer + offset++) = D_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < D_length; i++){
      long * val_Di = (long *) &(this->D[i]);
      long exp_Di = (((*val_Di)>>23)&255);
      if(exp_Di != 0)
        exp_Di += 1023-127;
      long sig_Di = *val_Di;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Di<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Di>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Di>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Di<<4) & 0xF0) | ((sig_Di>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Di>>4) & 0x7F;
      if(this->D[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      unsigned char * K_val = (unsigned char *) this->K;
      for( unsigned char i = 0; i < 9; i++){
      long * val_Ki = (long *) &(this->K[i]);
      long exp_Ki = (((*val_Ki)>>23)&255);
      if(exp_Ki != 0)
        exp_Ki += 1023-127;
      long sig_Ki = *val_Ki;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Ki<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Ki>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Ki>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Ki<<4) & 0xF0) | ((sig_Ki>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Ki>>4) & 0x7F;
      if(this->K[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      unsigned char * R_val = (unsigned char *) this->R;
      for( unsigned char i = 0; i < 9; i++){
      long * val_Ri = (long *) &(this->R[i]);
      long exp_Ri = (((*val_Ri)>>23)&255);
      if(exp_Ri != 0)
        exp_Ri += 1023-127;
      long sig_Ri = *val_Ri;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Ri<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Ri>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Ri>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Ri<<4) & 0xF0) | ((sig_Ri>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Ri>>4) & 0x7F;
      if(this->R[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      unsigned char * P_val = (unsigned char *) this->P;
      for( unsigned char i = 0; i < 12; i++){
      long * val_Pi = (long *) &(this->P[i]);
      long exp_Pi = (((*val_Pi)>>23)&255);
      if(exp_Pi != 0)
        exp_Pi += 1023-127;
      long sig_Pi = *val_Pi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Pi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Pi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Pi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Pi<<4) & 0xF0) | ((sig_Pi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Pi>>4) & 0x7F;
      if(this->P[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      union {
        unsigned long real;
        unsigned long base;
      } u_binning_x;
      u_binning_x.real = this->binning_x;
      *(outbuffer + offset + 0) = (u_binning_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_binning_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_binning_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_binning_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_x);
      union {
        unsigned long real;
        unsigned long base;
      } u_binning_y;
      u_binning_y.real = this->binning_y;
      *(outbuffer + offset + 0) = (u_binning_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_binning_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_binning_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_binning_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_y);
      offset += this->roi.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        unsigned long real;
        unsigned long base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      long * length_distortion_model = (long *)(inbuffer + offset);
      offset += 4;
      this->distortion_model = (inbuffer + offset);
      offset += *length_distortion_model;
      unsigned char D_lengthT = *(inbuffer + offset++);
      if(D_lengthT > D_length)
        this->D = (float*)realloc(this->D, D_lengthT * sizeof(float));
      offset += 3;
      D_length = D_lengthT;
      for( unsigned char i = 0; i < D_length; i++){
      unsigned long * val_st_D = (unsigned long*) &(this->st_D);
      offset += 3;
      *val_st_D = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_D |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_D |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_D |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_st_D = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_D |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_D !=0)
        *val_st_D |= ((exp_st_D)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_D = -this->st_D;
        memcpy( &(this->D[i]), &(this->st_D), sizeof(float));
      }
      unsigned char * K_val = (unsigned char *) this->K;
      for( unsigned char i = 0; i < 9; i++){
      unsigned long * val_Ki = (unsigned long*) &(this->K[i]);
      offset += 3;
      *val_Ki = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_Ki |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_Ki |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_Ki |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_Ki = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_Ki |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_Ki !=0)
        *val_Ki |= ((exp_Ki)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->K[i] = -this->K[i];
      }
      unsigned char * R_val = (unsigned char *) this->R;
      for( unsigned char i = 0; i < 9; i++){
      unsigned long * val_Ri = (unsigned long*) &(this->R[i]);
      offset += 3;
      *val_Ri = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_Ri |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_Ri |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_Ri |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_Ri = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_Ri |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_Ri !=0)
        *val_Ri |= ((exp_Ri)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->R[i] = -this->R[i];
      }
      unsigned char * P_val = (unsigned char *) this->P;
      for( unsigned char i = 0; i < 12; i++){
      unsigned long * val_Pi = (unsigned long*) &(this->P[i]);
      offset += 3;
      *val_Pi = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_Pi |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_Pi |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_Pi |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_Pi = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_Pi |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_Pi !=0)
        *val_Pi |= ((exp_Pi)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->P[i] = -this->P[i];
      }
      union {
        unsigned long real;
        unsigned long base;
      } u_binning_x;
      u_binning_x.base = 0;
      u_binning_x.base |= ((typeof(u_binning_x.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_binning_x.base |= ((typeof(u_binning_x.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_binning_x.base |= ((typeof(u_binning_x.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_binning_x.base |= ((typeof(u_binning_x.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->binning_x = u_binning_x.real;
      offset += sizeof(this->binning_x);
      union {
        unsigned long real;
        unsigned long base;
      } u_binning_y;
      u_binning_y.base = 0;
      u_binning_y.base |= ((typeof(u_binning_y.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_binning_y.base |= ((typeof(u_binning_y.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_binning_y.base |= ((typeof(u_binning_y.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_binning_y.base |= ((typeof(u_binning_y.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->binning_y = u_binning_y.real;
      offset += sizeof(this->binning_y);
      offset += this->roi.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/CameraInfo"; };

  };

}
#endif