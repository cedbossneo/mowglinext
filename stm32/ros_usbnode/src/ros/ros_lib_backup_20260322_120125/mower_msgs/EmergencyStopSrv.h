#ifndef _ROS_SERVICE_EmergencyStopSrv_h
#define _ROS_SERVICE_EmergencyStopSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

static const char EMERGENCYSTOPSRV[] = "mower_msgs/EmergencyStopSrv";

  class EmergencyStopSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _emergency_type;
      _emergency_type emergency;

    EmergencyStopSrvRequest():
      emergency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->emergency >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->emergency =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->emergency);
     return offset;
    }

    virtual const char * getType() override { return EMERGENCYSTOPSRV; };
    virtual const char * getMD5() override { return "00b85207f5d294f8ce1433699fc5397c"; };

  };

  class EmergencyStopSrvResponse : public ros::Msg
  {
    public:

    EmergencyStopSrvResponse()
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

    virtual const char * getType() override { return EMERGENCYSTOPSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class EmergencyStopSrv {
    public:
    typedef EmergencyStopSrvRequest Request;
    typedef EmergencyStopSrvResponse Response;
  };

}
#endif
