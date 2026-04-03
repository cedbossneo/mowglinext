#ifndef _ROS_SERVICE_PerimeterControlSrv_h
#define _ROS_SERVICE_PerimeterControlSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

static const char PERIMETERCONTROLSRV[] = "mower_msgs/PerimeterControlSrv";

  class PerimeterControlSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _listenOn_type;
      _listenOn_type listenOn;

    PerimeterControlSrvRequest():
      listenOn(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->listenOn >> (8 * 0)) & 0xFF;
      offset += sizeof(this->listenOn);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->listenOn =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->listenOn);
     return offset;
    }

    virtual const char * getType() override { return PERIMETERCONTROLSRV; };
    virtual const char * getMD5() override { return "b5c834610b09e83b616449810705d15b"; };

  };

  class PerimeterControlSrvResponse : public ros::Msg
  {
    public:

    PerimeterControlSrvResponse()
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

    virtual const char * getType() override { return PERIMETERCONTROLSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class PerimeterControlSrv {
    public:
    typedef PerimeterControlSrvRequest Request;
    typedef PerimeterControlSrvResponse Response;
  };

}
#endif
