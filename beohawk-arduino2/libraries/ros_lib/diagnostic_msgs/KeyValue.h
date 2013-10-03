#ifndef ros_KeyValue_h
#define ros_KeyValue_h

#include "WProgram.h"
#include "ros.h"

namespace diagnostic_msgs
{

  class KeyValue : public ros::Msg
  {
    public:
      unsigned char * key;
      unsigned char * value;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_key = (long *)(outbuffer + offset);
      *length_key = strlen( (const char*) this->key);
      offset += 4;
      memcpy(outbuffer + offset, this->key, *length_key);
      offset += *length_key;
      long * length_value = (long *)(outbuffer + offset);
      *length_value = strlen( (const char*) this->value);
      offset += 4;
      memcpy(outbuffer + offset, this->value, *length_value);
      offset += *length_value;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      long * length_key = (long *)(inbuffer + offset);
      offset += 4;
      this->key = (inbuffer + offset);
      offset += *length_key;
      long * length_value = (long *)(inbuffer + offset);
      offset += 4;
      this->value = (inbuffer + offset);
      offset += *length_value;
     return offset;
    }

    const char * getType(){ return "diagnostic_msgs/KeyValue"; };

  };

}
#endif