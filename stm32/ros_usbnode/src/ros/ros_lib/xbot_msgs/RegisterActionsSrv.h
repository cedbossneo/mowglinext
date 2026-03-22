#ifndef _ROS_SERVICE_RegisterActionsSrv_h
#define _ROS_SERVICE_RegisterActionsSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "xbot_msgs/ActionInfo.h"

namespace xbot_msgs
{

static const char REGISTERACTIONSSRV[] = "xbot_msgs/RegisterActionsSrv";

  class RegisterActionsSrvRequest : public ros::Msg
  {
    public:
      typedef const char* _node_prefix_type;
      _node_prefix_type node_prefix;
      uint32_t actions_length;
      typedef xbot_msgs::ActionInfo _actions_type;
      _actions_type st_actions;
      _actions_type * actions;

    RegisterActionsSrvRequest():
      node_prefix(""),
      actions_length(0), st_actions(), actions(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_node_prefix = strlen(this->node_prefix);
      varToArr(outbuffer + offset, length_node_prefix);
      offset += 4;
      memcpy(outbuffer + offset, this->node_prefix, length_node_prefix);
      offset += length_node_prefix;
      *(outbuffer + offset + 0) = (this->actions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->actions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->actions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->actions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actions_length);
      for( uint32_t i = 0; i < actions_length; i++){
      offset += this->actions[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_node_prefix;
      arrToVar(length_node_prefix, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_prefix; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_prefix-1]=0;
      this->node_prefix = (char *)(inbuffer + offset-1);
      offset += length_node_prefix;
      uint32_t actions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      actions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      actions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      actions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->actions_length);
      if(actions_lengthT > actions_length)
        this->actions = (xbot_msgs::ActionInfo*)realloc(this->actions, actions_lengthT * sizeof(xbot_msgs::ActionInfo));
      actions_length = actions_lengthT;
      for( uint32_t i = 0; i < actions_length; i++){
      offset += this->st_actions.deserialize(inbuffer + offset);
        memcpy( &(this->actions[i]), &(this->st_actions), sizeof(xbot_msgs::ActionInfo));
      }
     return offset;
    }

    virtual const char * getType() override { return REGISTERACTIONSSRV; };
    virtual const char * getMD5() override { return "0d2ab873c76b3d03247f8862ec188635"; };

  };

  class RegisterActionsSrvResponse : public ros::Msg
  {
    public:

    RegisterActionsSrvResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return REGISTERACTIONSSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class RegisterActionsSrv {
    public:
    typedef RegisterActionsSrvRequest Request;
    typedef RegisterActionsSrvResponse Response;
  };

}
#endif
