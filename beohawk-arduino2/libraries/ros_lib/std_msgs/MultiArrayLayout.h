#ifndef ros_MultiArrayLayout_h
#define ros_MultiArrayLayout_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/MultiArrayDimension.h"

namespace std_msgs
{

  class MultiArrayLayout : public ros::Msg
  {
    public:
      unsigned char dim_length;
      std_msgs::MultiArrayDimension st_dim;
      std_msgs::MultiArrayDimension * dim;
      unsigned long data_offset;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      *(outbuffer + offset++) = dim_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < dim_length; i++){
      offset += this->dim[i].serialize(outbuffer + offset);
      }
      union {
        unsigned long real;
        unsigned long base;
      } u_data_offset;
      u_data_offset.real = this->data_offset;
      *(outbuffer + offset + 0) = (u_data_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned char dim_lengthT = *(inbuffer + offset++);
      if(dim_lengthT > dim_length)
        this->dim = (std_msgs::MultiArrayDimension*)realloc(this->dim, dim_lengthT * sizeof(std_msgs::MultiArrayDimension));
      offset += 3;
      dim_length = dim_lengthT;
      for( unsigned char i = 0; i < dim_length; i++){
      offset += this->st_dim.deserialize(inbuffer + offset);
        memcpy( &(this->dim[i]), &(this->st_dim), sizeof(std_msgs::MultiArrayDimension));
      }
      union {
        unsigned long real;
        unsigned long base;
      } u_data_offset;
      u_data_offset.base = 0;
      u_data_offset.base |= ((typeof(u_data_offset.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_offset.base |= ((typeof(u_data_offset.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_offset.base |= ((typeof(u_data_offset.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_offset.base |= ((typeof(u_data_offset.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_offset = u_data_offset.real;
      offset += sizeof(this->data_offset);
     return offset;
    }

    const char * getType(){ return "std_msgs/MultiArrayLayout"; };

  };

}
#endif