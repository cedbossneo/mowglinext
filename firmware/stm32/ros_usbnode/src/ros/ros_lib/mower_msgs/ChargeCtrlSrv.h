#ifndef _ROS_SERVICE_ChargeCtrlSrv_h
#define _ROS_SERVICE_ChargeCtrlSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

static const char CHARGECTRLSRV[] = "mowgli/ChargeCtrlSrv";

  class ChargeCtrlSrvRequest : public ros::Msg
  {
    public:
      typedef float _eoc_type;
      _eoc_type eoc;

    ChargeCtrlSrvRequest():
      eoc(0.0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_eoc;
      u_eoc.real = this->eoc;
      *(outbuffer + offset + 0) = (u_eoc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_eoc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_eoc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_eoc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eoc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_eoc;
      u_eoc.base = 0;
      u_eoc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_eoc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_eoc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_eoc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->eoc = u_eoc.real;
      offset += sizeof(this->eoc);
     return offset;
    }

    virtual const char * getType() override { return CHARGECTRLSRV; };
    virtual const char * getMD5() override { return "103ccada3ef875e8caf5690afaf72f47"; };

  };

  class ChargeCtrlSrvResponse : public ros::Msg
  {
    public:

    ChargeCtrlSrvResponse()
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

    virtual const char * getType() override { return CHARGECTRLSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ChargeCtrlSrv {
    public:
    typedef ChargeCtrlSrvRequest Request;
    typedef ChargeCtrlSrvResponse Response;
  };

}
#endif
