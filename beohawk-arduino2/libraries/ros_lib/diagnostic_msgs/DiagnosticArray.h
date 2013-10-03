#ifndef ros_DiagnosticArray_h
#define ros_DiagnosticArray_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

namespace diagnostic_msgs
{

  class DiagnosticArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char status_length;
      diagnostic_msgs::DiagnosticStatus st_status;
      diagnostic_msgs::DiagnosticStatus * status;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = status_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      unsigned char status_lengthT = *(inbuffer + offset++);
      if(status_lengthT > status_length)
        this->status = (diagnostic_msgs::DiagnosticStatus*)realloc(this->status, status_lengthT * sizeof(diagnostic_msgs::DiagnosticStatus));
      offset += 3;
      status_length = status_lengthT;
      for( unsigned char i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(diagnostic_msgs::DiagnosticStatus));
      }
     return offset;
    }

    const char * getType(){ return "diagnostic_msgs/DiagnosticArray"; };

  };

}
#endif