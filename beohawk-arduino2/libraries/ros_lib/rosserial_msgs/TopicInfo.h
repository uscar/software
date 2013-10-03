#ifndef ros_TopicInfo_h
#define ros_TopicInfo_h

#include "WProgram.h"
#include "ros.h"

namespace rosserial_msgs
{

  class TopicInfo : public ros::Msg
  {
    public:
      unsigned int topic_id;
      unsigned char * topic_name;
      unsigned char * message_type;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned int real;
        unsigned int base;
      } u_topic_id;
      u_topic_id.real = this->topic_id;
      *(outbuffer + offset + 0) = (u_topic_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_topic_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->topic_id);
      long * length_topic_name = (long *)(outbuffer + offset);
      *length_topic_name = strlen( (const char*) this->topic_name);
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, *length_topic_name);
      offset += *length_topic_name;
      long * length_message_type = (long *)(outbuffer + offset);
      *length_message_type = strlen( (const char*) this->message_type);
      offset += 4;
      memcpy(outbuffer + offset, this->message_type, *length_message_type);
      offset += *length_message_type;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned int real;
        unsigned int base;
      } u_topic_id;
      u_topic_id.base = 0;
      u_topic_id.base |= ((typeof(u_topic_id.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_topic_id.base |= ((typeof(u_topic_id.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->topic_id = u_topic_id.real;
      offset += sizeof(this->topic_id);
      long * length_topic_name = (long *)(inbuffer + offset);
      offset += 4;
      this->topic_name = (inbuffer + offset);
      offset += *length_topic_name;
      long * length_message_type = (long *)(inbuffer + offset);
      offset += 4;
      this->message_type = (inbuffer + offset);
      offset += *length_message_type;
     return offset;
    }

    const char * getType(){ return "rosserial_msgs/TopicInfo"; };

  };

}
#endif