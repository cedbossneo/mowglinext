#ifndef _ROS_SERVICE_MowerControlSrv_h
#define _ROS_SERVICE_MowerControlSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

static const char MOWERCONTROLSRV[] = "mower_msgs/MowerControlSrv";

  class MowerControlSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _mow_enabled_type;
      _mow_enabled_type mow_enabled;
      typedef uint8_t _mow_direction_type;
      _mow_direction_type mow_direction;

    MowerControlSrvRequest():
      mow_enabled(0),
      mow_direction(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mow_enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mow_enabled);
      *(outbuffer + offset + 0) = (this->mow_direction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mow_direction);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->mow_enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mow_enabled);
      this->mow_direction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mow_direction);
     return offset;
    }

    virtual const char * getType() override { return MOWERCONTROLSRV; };
    virtual const char * getMD5() override { return "94d32eb6abf74150f0ffee85ec2e3056"; };

  };

  class MowerControlSrvResponse : public ros::Msg
  {
    public:

    MowerControlSrvResponse()
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

    virtual const char * getType() override { return MOWERCONTROLSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class MowerControlSrv {
    public:
    typedef MowerControlSrvRequest Request;
    typedef MowerControlSrvResponse Response;
  };

}
#endif
