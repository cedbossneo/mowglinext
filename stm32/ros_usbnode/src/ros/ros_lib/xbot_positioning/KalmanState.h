#ifndef _ROS_xbot_positioning_KalmanState_h
#define _ROS_xbot_positioning_KalmanState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_positioning
{

  class KalmanState : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _theta_type;
      _theta_type theta;
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vr_type;
      _vr_type vr;

    KalmanState():
      x(0),
      y(0),
      theta(0),
      vx(0),
      vr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta);
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vr));
     return offset;
    }

    virtual const char * getType() override { return "xbot_positioning/KalmanState"; };
    virtual const char * getMD5() override { return "61df9a69cd63d907340181c20aa2d1c5"; };

  };

}
#endif
