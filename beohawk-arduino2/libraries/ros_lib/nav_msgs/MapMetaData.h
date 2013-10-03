#ifndef ros_MapMetaData_h
#define ros_MapMetaData_h

#include "WProgram.h"
#include "ros.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"

namespace nav_msgs
{

  class MapMetaData : public ros::Msg
  {
    public:
      ros::Time map_load_time;
      float resolution;
      unsigned long width;
      unsigned long height;
      geometry_msgs::Pose origin;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.real = this->map_load_time.sec;
      *(outbuffer + offset + 0) = (u_sec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_load_time.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.real = this->map_load_time.nsec;
      *(outbuffer + offset + 0) = (u_nsec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nsec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nsec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nsec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_load_time.nsec);
      union {
        float real;
        unsigned long base;
      } u_resolution;
      u_resolution.real = this->resolution;
      *(outbuffer + offset + 0) = (u_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resolution.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->resolution);
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
      offset += this->origin.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.base = 0;
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->map_load_time.sec = u_sec.real;
      offset += sizeof(this->map_load_time.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.base = 0;
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->map_load_time.nsec = u_nsec.real;
      offset += sizeof(this->map_load_time.nsec);
      union {
        float real;
        unsigned long base;
      } u_resolution;
      u_resolution.base = 0;
      u_resolution.base |= ((typeof(u_resolution.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_resolution.base |= ((typeof(u_resolution.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_resolution.base |= ((typeof(u_resolution.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_resolution.base |= ((typeof(u_resolution.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->resolution = u_resolution.real;
      offset += sizeof(this->resolution);
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
      offset += this->origin.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav_msgs/MapMetaData"; };

  };

}
#endif