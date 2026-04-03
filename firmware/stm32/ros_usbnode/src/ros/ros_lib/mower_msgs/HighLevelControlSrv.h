#ifndef _ROS_SERVICE_HighLevelControlSrv_h
#define _ROS_SERVICE_HighLevelControlSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

static const char HIGHLEVELCONTROLSRV[] = "mower_msgs/HighLevelControlSrv";

  class HighLevelControlSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _command_type;
      _command_type command;
      enum { COMMAND_START = 1 };
      enum { COMMAND_HOME = 2 };
      enum { COMMAND_S1 = 3 };
      enum { COMMAND_S2 = 4 };
      enum { COMMAND_RESET_EMERGENCY = 254 };
      enum { COMMAND_DELETE_MAPS = 255 };

    HighLevelControlSrvRequest():
      command(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->command);
     return offset;
    }

    virtual const char * getType() override { return HIGHLEVELCONTROLSRV; };
    virtual const char * getMD5() override { return "0c9afcf75aa7a3bc23f835cf875e0cde"; };

  };

  class HighLevelControlSrvResponse : public ros::Msg
  {
    public:

    HighLevelControlSrvResponse()
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

    virtual const char * getType() override { return HIGHLEVELCONTROLSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class HighLevelControlSrv {
    public:
    typedef HighLevelControlSrvRequest Request;
    typedef HighLevelControlSrvResponse Response;
  };

}
#endif
