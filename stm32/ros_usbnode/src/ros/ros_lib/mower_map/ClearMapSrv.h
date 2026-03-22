#ifndef _ROS_SERVICE_ClearMapSrv_h
#define _ROS_SERVICE_ClearMapSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_map
{

static const char CLEARMAPSRV[] = "mower_map/ClearMapSrv";

  class ClearMapSrvRequest : public ros::Msg
  {
    public:

    ClearMapSrvRequest()
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

    virtual const char * getType() override { return CLEARMAPSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ClearMapSrvResponse : public ros::Msg
  {
    public:

    ClearMapSrvResponse()
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

    virtual const char * getType() override { return CLEARMAPSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ClearMapSrv {
    public:
    typedef ClearMapSrvRequest Request;
    typedef ClearMapSrvResponse Response;
  };

}
#endif
