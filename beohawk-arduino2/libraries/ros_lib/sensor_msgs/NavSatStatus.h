#ifndef ros_NavSatStatus_h
#define ros_NavSatStatus_h

#include "WProgram.h"
#include "ros.h"

namespace sensor_msgs
{

  class NavSatStatus : public ros::Msg
  {
    public:
      signed char status;
      unsigned int service;
      enum { STATUS_NO_FIX = -1 };
      enum { STATUS_FIX = 0 };
      enum { STATUS_SBAS_FIX = 1 };
      enum { STATUS_GBAS_FIX = 2 };
      enum { SERVICE_GPS = 1 };
      enum { SERVICE_GLONASS = 2 };
      enum { SERVICE_COMPASS = 4 };
      enum { SERVICE_GALILEO = 8 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        signed char real;
        unsigned char base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      union {
        unsigned int real;
        unsigned int base;
      } u_service;
      u_service.real = this->service;
      *(outbuffer + offset + 0) = (u_service.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_service.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->service);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        signed char real;
        unsigned char base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((typeof(u_status.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
      union {
        unsigned int real;
        unsigned int base;
      } u_service;
      u_service.base = 0;
      u_service.base |= ((typeof(u_service.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_service.base |= ((typeof(u_service.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->service = u_service.real;
      offset += sizeof(this->service);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/NavSatStatus"; };

  };

}
#endif