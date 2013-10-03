#ifndef ros_ChannelFloat32_h
#define ros_ChannelFloat32_h

#include "WProgram.h"
#include "ros.h"

namespace sensor_msgs
{

  class ChannelFloat32 : public ros::Msg
  {
    public:
      unsigned char * name;
      unsigned char values_length;
      float st_values;
      float * values;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_name = (long *)(outbuffer + offset);
      *length_name = strlen( (const char*) this->name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, *length_name);
      offset += *length_name;
      *(outbuffer + offset++) = values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < values_length; i++){
      union {
        float real;
        unsigned long base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      long * length_name = (long *)(inbuffer + offset);
      offset += 4;
      this->name = (inbuffer + offset);
      offset += *length_name;
      unsigned char values_lengthT = *(inbuffer + offset++);
      if(values_lengthT > values_length)
        this->values = (float*)realloc(this->values, values_lengthT * sizeof(float));
      offset += 3;
      values_length = values_lengthT;
      for( unsigned char i = 0; i < values_length; i++){
      union {
        float real;
        unsigned long base;
      } u_st_values;
      u_st_values.base = 0;
      u_st_values.base |= ((typeof(u_st_values.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_values.base |= ((typeof(u_st_values.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_values.base |= ((typeof(u_st_values.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_values.base |= ((typeof(u_st_values.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_values = u_st_values.real;
      offset += sizeof(this->st_values);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/ChannelFloat32"; };

  };

}
#endif