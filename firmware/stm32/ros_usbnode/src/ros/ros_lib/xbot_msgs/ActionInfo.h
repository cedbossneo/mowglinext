#ifndef _ROS_xbot_msgs_ActionInfo_h
#define _ROS_xbot_msgs_ActionInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_msgs
{

  class ActionInfo : public ros::Msg
  {
    public:
      typedef const char* _action_id_type;
      _action_id_type action_id;
      typedef const char* _action_name_type;
      _action_name_type action_name;
      typedef bool _enabled_type;
      _enabled_type enabled;

    ActionInfo():
      action_id(""),
      action_name(""),
      enabled(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_action_id = strlen(this->action_id);
      varToArr(outbuffer + offset, length_action_id);
      offset += 4;
      memcpy(outbuffer + offset, this->action_id, length_action_id);
      offset += length_action_id;
      uint32_t length_action_name = strlen(this->action_name);
      varToArr(outbuffer + offset, length_action_name);
      offset += 4;
      memcpy(outbuffer + offset, this->action_name, length_action_name);
      offset += length_action_name;
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.real = this->enabled;
      *(outbuffer + offset + 0) = (u_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_action_id;
      arrToVar(length_action_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_id-1]=0;
      this->action_id = (char *)(inbuffer + offset-1);
      offset += length_action_id;
      uint32_t length_action_name;
      arrToVar(length_action_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_name-1]=0;
      this->action_name = (char *)(inbuffer + offset-1);
      offset += length_action_name;
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.base = 0;
      u_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enabled = u_enabled.real;
      offset += sizeof(this->enabled);
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/ActionInfo"; };
    virtual const char * getMD5() override { return "98dc1981a42cdfe360677d00f0e0730a"; };

  };

}
#endif
