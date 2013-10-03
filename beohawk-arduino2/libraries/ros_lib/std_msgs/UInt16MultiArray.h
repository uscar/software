#ifndef ros_UInt16MultiArray_h
#define ros_UInt16MultiArray_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/MultiArrayLayout.h"

namespace std_msgs
{

  class UInt16MultiArray : public ros::Msg
  {
    public:
      std_msgs::MultiArrayLayout layout;
      unsigned char data_length;
      unsigned int st_data;
      unsigned int * data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->layout.serialize(outbuffer + offset);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < data_length; i++){
      union {
        unsigned int real;
        unsigned int base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->layout.deserialize(inbuffer + offset);
      unsigned char data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (unsigned int*)realloc(this->data, data_lengthT * sizeof(unsigned int));
      offset += 3;
      data_length = data_lengthT;
      for( unsigned char i = 0; i < data_length; i++){
      union {
        unsigned int real;
        unsigned int base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((typeof(u_st_data.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((typeof(u_st_data.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(unsigned int));
      }
     return offset;
    }

    const char * getType(){ return "std_msgs/UInt16MultiArray"; };

  };

}
#endif