#ifndef _ROS_SERVICE_ClearNavPointSrv_h
#define _ROS_SERVICE_ClearNavPointSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_map
{

static const char CLEARNAVPOINTSRV[] = "mower_map/ClearNavPointSrv";

  class ClearNavPointSrvRequest : public ros::Msg
  {
    public:

    ClearNavPointSrvRequest()
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

    virtual const char * getType() override { return CLEARNAVPOINTSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ClearNavPointSrvResponse : public ros::Msg
  {
    public:

    ClearNavPointSrvResponse()
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

    virtual const char * getType() override { return CLEARNAVPOINTSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ClearNavPointSrv {
    public:
    typedef ClearNavPointSrvRequest Request;
    typedef ClearNavPointSrvResponse Response;
  };

}
#endif
