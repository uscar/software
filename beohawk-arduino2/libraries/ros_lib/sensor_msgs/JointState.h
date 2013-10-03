#ifndef ros_JointState_h
#define ros_JointState_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class JointState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char name_length;
      unsigned char* st_name;
      unsigned char* * name;
      unsigned char position_length;
      float st_position;
      float * position;
      unsigned char velocity_length;
      float st_velocity;
      float * velocity;
      unsigned char effort_length;
      float st_effort;
      float * effort;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = name_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < name_length; i++){
      long * length_namei = (long *)(outbuffer + offset);
      *length_namei = strlen( (const char*) this->name[i]);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], *length_namei);
      offset += *length_namei;
      }
      *(outbuffer + offset++) = position_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < position_length; i++){
      long * val_positioni = (long *) &(this->position[i]);
      long exp_positioni = (((*val_positioni)>>23)&255);
      if(exp_positioni != 0)
        exp_positioni += 1023-127;
      long sig_positioni = *val_positioni;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_positioni<<5) & 0xff;
      *(outbuffer + offset++) = (sig_positioni>>3) & 0xff;
      *(outbuffer + offset++) = (sig_positioni>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_positioni<<4) & 0xF0) | ((sig_positioni>>19)&0x0F);
      *(outbuffer + offset++) = (exp_positioni>>4) & 0x7F;
      if(this->position[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      *(outbuffer + offset++) = velocity_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < velocity_length; i++){
      long * val_velocityi = (long *) &(this->velocity[i]);
      long exp_velocityi = (((*val_velocityi)>>23)&255);
      if(exp_velocityi != 0)
        exp_velocityi += 1023-127;
      long sig_velocityi = *val_velocityi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_velocityi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_velocityi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_velocityi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_velocityi<<4) & 0xF0) | ((sig_velocityi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_velocityi>>4) & 0x7F;
      if(this->velocity[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      *(outbuffer + offset++) = effort_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < effort_length; i++){
      long * val_efforti = (long *) &(this->effort[i]);
      long exp_efforti = (((*val_efforti)>>23)&255);
      if(exp_efforti != 0)
        exp_efforti += 1023-127;
      long sig_efforti = *val_efforti;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_efforti<<5) & 0xff;
      *(outbuffer + offset++) = (sig_efforti>>3) & 0xff;
      *(outbuffer + offset++) = (sig_efforti>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_efforti<<4) & 0xF0) | ((sig_efforti>>19)&0x0F);
      *(outbuffer + offset++) = (exp_efforti>>4) & 0x7F;
      if(this->effort[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      unsigned char name_lengthT = *(inbuffer + offset++);
      if(name_lengthT > name_length)
        this->name = (unsigned char**)realloc(this->name, name_lengthT * sizeof(unsigned char*));
      offset += 3;
      name_length = name_lengthT;
      for( unsigned char i = 0; i < name_length; i++){
      long * length_st_name = (long *)(inbuffer + offset);
      offset += 4;
      this->st_name = (inbuffer + offset);
      offset += *length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(unsigned char*));
      }
      unsigned char position_lengthT = *(inbuffer + offset++);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      offset += 3;
      position_length = position_lengthT;
      for( unsigned char i = 0; i < position_length; i++){
      unsigned long * val_st_position = (unsigned long*) &(this->st_position);
      offset += 3;
      *val_st_position = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_position |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_position |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_position |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_st_position = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_position |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_position !=0)
        *val_st_position |= ((exp_st_position)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_position = -this->st_position;
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
      unsigned char velocity_lengthT = *(inbuffer + offset++);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      offset += 3;
      velocity_length = velocity_lengthT;
      for( unsigned char i = 0; i < velocity_length; i++){
      unsigned long * val_st_velocity = (unsigned long*) &(this->st_velocity);
      offset += 3;
      *val_st_velocity = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_velocity |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_velocity |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_velocity |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_st_velocity = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_velocity |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_velocity !=0)
        *val_st_velocity |= ((exp_st_velocity)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_velocity = -this->st_velocity;
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
      }
      unsigned char effort_lengthT = *(inbuffer + offset++);
      if(effort_lengthT > effort_length)
        this->effort = (float*)realloc(this->effort, effort_lengthT * sizeof(float));
      offset += 3;
      effort_length = effort_lengthT;
      for( unsigned char i = 0; i < effort_length; i++){
      unsigned long * val_st_effort = (unsigned long*) &(this->st_effort);
      offset += 3;
      *val_st_effort = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_effort |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_effort |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_effort |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_st_effort = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_effort |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_effort !=0)
        *val_st_effort |= ((exp_st_effort)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_effort = -this->st_effort;
        memcpy( &(this->effort[i]), &(this->st_effort), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/JointState"; };

  };

}
#endif