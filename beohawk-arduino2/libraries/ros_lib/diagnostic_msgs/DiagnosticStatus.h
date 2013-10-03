#ifndef ros_DiagnosticStatus_h
#define ros_DiagnosticStatus_h

#include "WProgram.h"
#include "ros.h"
#include "diagnostic_msgs/KeyValue.h"

namespace diagnostic_msgs
{

  class DiagnosticStatus : public ros::Msg
  {
    public:
      byte level;
      unsigned char * name;
      unsigned char * message;
      unsigned char * hardware_id;
      unsigned char values_length;
      diagnostic_msgs::KeyValue st_values;
      diagnostic_msgs::KeyValue * values;
      enum { OK = 0 };
      enum { WARN = 1 };
      enum { ERROR = 2 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        byte real;
        unsigned char base;
      } u_level;
      u_level.real = this->level;
      *(outbuffer + offset + 0) = (u_level.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      long * length_name = (long *)(outbuffer + offset);
      *length_name = strlen( (const char*) this->name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, *length_name);
      offset += *length_name;
      long * length_message = (long *)(outbuffer + offset);
      *length_message = strlen( (const char*) this->message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, *length_message);
      offset += *length_message;
      long * length_hardware_id = (long *)(outbuffer + offset);
      *length_hardware_id = strlen( (const char*) this->hardware_id);
      offset += 4;
      memcpy(outbuffer + offset, this->hardware_id, *length_hardware_id);
      offset += *length_hardware_id;
      *(outbuffer + offset++) = values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < values_length; i++){
      offset += this->values[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        byte real;
        unsigned char base;
      } u_level;
      u_level.base = 0;
      u_level.base |= ((typeof(u_level.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->level = u_level.real;
      offset += sizeof(this->level);
      long * length_name = (long *)(inbuffer + offset);
      offset += 4;
      this->name = (inbuffer + offset);
      offset += *length_name;
      long * length_message = (long *)(inbuffer + offset);
      offset += 4;
      this->message = (inbuffer + offset);
      offset += *length_message;
      long * length_hardware_id = (long *)(inbuffer + offset);
      offset += 4;
      this->hardware_id = (inbuffer + offset);
      offset += *length_hardware_id;
      unsigned char values_lengthT = *(inbuffer + offset++);
      if(values_lengthT > values_length)
        this->values = (diagnostic_msgs::KeyValue*)realloc(this->values, values_lengthT * sizeof(diagnostic_msgs::KeyValue));
      offset += 3;
      values_length = values_lengthT;
      for( unsigned char i = 0; i < values_length; i++){
      offset += this->st_values.deserialize(inbuffer + offset);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(diagnostic_msgs::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "diagnostic_msgs/DiagnosticStatus"; };

  };

}
#endif