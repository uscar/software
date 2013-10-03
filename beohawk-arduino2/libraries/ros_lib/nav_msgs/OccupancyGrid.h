#ifndef ros_OccupancyGrid_h
#define ros_OccupancyGrid_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

namespace nav_msgs
{

  class OccupancyGrid : public ros::Msg
  {
    public:
      std_msgs::Header header;
      nav_msgs::MapMetaData info;
      unsigned char data_length;
      signed char st_data;
      signed char * data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->info.serialize(outbuffer + offset);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < data_length; i++){
      union {
        signed char real;
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
      offset += this->info.deserialize(inbuffer + offset);
      unsigned char data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (signed char*)realloc(this->data, data_lengthT * sizeof(signed char));
      offset += 3;
      data_length = data_lengthT;
      for( unsigned char i = 0; i < data_length; i++){
      union {
        signed char real;
        unsigned char base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((typeof(u_st_data.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(signed char));
      }
     return offset;
    }

    const char * getType(){ return "nav_msgs/OccupancyGrid"; };

  };

}
#endif