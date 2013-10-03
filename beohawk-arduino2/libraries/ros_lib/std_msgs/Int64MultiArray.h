#ifndef ros_Int64MultiArray_h
#define ros_Int64MultiArray_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/MultiArrayLayout.h"

namespace std_msgs
{

  class Int64MultiArray : public ros::Msg
  {
    public:
      std_msgs::MultiArrayLayout layout;
      unsigned char data_length;
      long st_data;
      long * data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->layout.serialize(outbuffer + offset);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < data_length; i++){
      *(outbuffer + offset++) = (data[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset++) = (data[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset++) = (data[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset++) = (data[i] >> (8 * 3)) & 0xFF;
      *(outbuffer + offset++) = (data[i] > 0) ? 0: 255;
      *(outbuffer + offset++) = (data[i] > 0) ? 0: 255;
      *(outbuffer + offset++) = (data[i] > 0) ? 0: 255;
      *(outbuffer + offset++) = (data[i] > 0) ? 0: 255;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->layout.deserialize(inbuffer + offset);
      unsigned char data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (long*)realloc(this->data, data_lengthT * sizeof(long));
      offset += 3;
      data_length = data_lengthT;
      for( unsigned char i = 0; i < data_length; i++){
      st_data = 0;
      st_data += ((long)(*(inbuffer + offset++))) >> (8 * 0);
      st_data += ((long)(*(inbuffer + offset++))) >> (8 * 1);
      st_data += ((long)(*(inbuffer + offset++))) >> (8 * 2);
      st_data += ((long)(*(inbuffer + offset++))) >> (8 * 3);
      offset += 4;
        memcpy( &(this->data[i]), &(this->st_data), sizeof(long));
      }
     return offset;
    }

    const char * getType(){ return "std_msgs/Int64MultiArray"; };

  };

}
#endif