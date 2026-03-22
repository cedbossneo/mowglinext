#ifndef _ROS_mower_msgs_ImuRaw_h
#define _ROS_mower_msgs_ImuRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

  class ImuRaw : public ros::Msg
  {
    public:
      typedef uint16_t _dt_type;
      _dt_type dt;
      typedef float _ax_type;
      _ax_type ax;
      typedef float _ay_type;
      _ay_type ay;
      typedef float _az_type;
      _az_type az;
      typedef float _gx_type;
      _gx_type gx;
      typedef float _gy_type;
      _gy_type gy;
      typedef float _gz_type;
      _gz_type gz;
      typedef float _mx_type;
      _mx_type mx;
      typedef float _my_type;
      _my_type my;
      typedef float _mz_type;
      _mz_type mz;

    ImuRaw():
      dt(0),
      ax(0),
      ay(0),
      az(0),
      gx(0),
      gy(0),
      gz(0),
      mx(0),
      my(0),
      mz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dt >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dt);
      offset += serializeAvrFloat64(outbuffer + offset, this->ax);
      offset += serializeAvrFloat64(outbuffer + offset, this->ay);
      offset += serializeAvrFloat64(outbuffer + offset, this->az);
      offset += serializeAvrFloat64(outbuffer + offset, this->gx);
      offset += serializeAvrFloat64(outbuffer + offset, this->gy);
      offset += serializeAvrFloat64(outbuffer + offset, this->gz);
      offset += serializeAvrFloat64(outbuffer + offset, this->mx);
      offset += serializeAvrFloat64(outbuffer + offset, this->my);
      offset += serializeAvrFloat64(outbuffer + offset, this->mz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->dt =  ((uint16_t) (*(inbuffer + offset)));
      this->dt |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->dt);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ax));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ay));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->az));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->my));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mz));
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/ImuRaw"; };
    virtual const char * getMD5() override { return "0352b07cbe7d2a21d2573179fd87547d"; };

  };

}
#endif
