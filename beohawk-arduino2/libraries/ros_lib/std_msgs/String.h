#ifndef ros_String_h
#define ros_String_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class String : public ros::Msg
  {
    public:
      unsigned char * data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_data = (long *)(outbuffer + offset);
      *length_data = strlen( (const char*) this->data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, *length_data);
      offset += *length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      long * length_data = (long *)(inbuffer + offset);
      offset += 4;
      this->data = (inbuffer + offset);
      offset += *length_data;
     return offset;
    }

    const char * getType(){ return "std_msgs/String"; };

  };

}
#endif