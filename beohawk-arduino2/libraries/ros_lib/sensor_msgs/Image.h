#ifndef ros_Image_h
#define ros_Image_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class Image : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned long height;
      unsigned long width;
      unsigned char * encoding;
      unsigned char is_bigendian;
      unsigned long step;
      unsigned char data_length;
      unsigned char st_data;
      unsigned char * data;

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
      long * length_encoding = (long *)(outbuffer + offset);
      *length_encoding = strlen( (const char*) this->encoding);
      offset += 4;
      memcpy(outbuffer + offset, this->encoding, *length_encoding);
      offset += *length_encoding;
      union {
        unsigned char real;
        unsigned char base;
      } u_is_bigendian;
      u_is_bigendian.real = this->is_bigendian;
      *(outbuffer + offset + 0) = (u_is_bigendian.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_bigendian);
      union {
        unsigned long real;
        unsigned long base;
      } u_step;
      u_step.real = this->step;
      *(outbuffer + offset + 0) = (u_step.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_step.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_step.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_step.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->step);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < data_length; i++){
      union {
        unsigned char real;
        unsigned char base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
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
      long * length_encoding = (long *)(inbuffer + offset);
      offset += 4;
      this->encoding = (inbuffer + offset);
      offset += *length_encoding;
      union {
        unsigned char real;
        unsigned char base;
      } u_is_bigendian;
      u_is_bigendian.base = 0;
      u_is_bigendian.base |= ((typeof(u_is_bigendian.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_bigendian = u_is_bigendian.real;
      offset += sizeof(this->is_bigendian);
      union {
        unsigned long real;
        unsigned long base;
      } u_step;
      u_step.base = 0;
      u_step.base |= ((typeof(u_step.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_step.base |= ((typeof(u_step.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_step.base |= ((typeof(u_step.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_step.base |= ((typeof(u_step.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->step = u_step.real;
      offset += sizeof(this->step);
      unsigned char data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (unsigned char*)realloc(this->data, data_lengthT * sizeof(unsigned char));
      offset += 3;
      data_length = data_lengthT;
      for( unsigned char i = 0; i < data_length; i++){
      union {
        unsigned char real;
        unsigned char base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((typeof(u_st_data.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(unsigned char));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/Image"; };

  };

}
#endif