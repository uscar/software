#ifndef ros_CompressedImage_h
#define ros_CompressedImage_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class CompressedImage : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char * format;
      unsigned char data_length;
      unsigned char st_data;
      unsigned char * data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      long * length_format = (long *)(outbuffer + offset);
      *length_format = strlen( (const char*) this->format);
      offset += 4;
      memcpy(outbuffer + offset, this->format, *length_format);
      offset += *length_format;
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
      long * length_format = (long *)(inbuffer + offset);
      offset += 4;
      this->format = (inbuffer + offset);
      offset += *length_format;
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

    const char * getType(){ return "sensor_msgs/CompressedImage"; };

  };

}
#endif