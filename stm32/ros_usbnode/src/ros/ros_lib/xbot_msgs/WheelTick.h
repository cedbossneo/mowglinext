#ifndef _ROS_xbot_msgs_WheelTick_h
#define _ROS_xbot_msgs_WheelTick_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace xbot_msgs
{

  class WheelTick : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef uint32_t _wheel_tick_factor_type;
      _wheel_tick_factor_type wheel_tick_factor;
      typedef uint8_t _valid_wheels_type;
      _valid_wheels_type valid_wheels;
      typedef uint8_t _wheel_direction_fl_type;
      _wheel_direction_fl_type wheel_direction_fl;
      typedef uint32_t _wheel_ticks_fl_type;
      _wheel_ticks_fl_type wheel_ticks_fl;
      typedef uint8_t _wheel_direction_fr_type;
      _wheel_direction_fr_type wheel_direction_fr;
      typedef uint32_t _wheel_ticks_fr_type;
      _wheel_ticks_fr_type wheel_ticks_fr;
      typedef uint8_t _wheel_direction_rl_type;
      _wheel_direction_rl_type wheel_direction_rl;
      typedef uint32_t _wheel_ticks_rl_type;
      _wheel_ticks_rl_type wheel_ticks_rl;
      typedef uint8_t _wheel_direction_rr_type;
      _wheel_direction_rr_type wheel_direction_rr;
      typedef uint32_t _wheel_ticks_rr_type;
      _wheel_ticks_rr_type wheel_ticks_rr;
      enum { WHEEL_VALID_FL = 1 };
      enum { WHEEL_VALID_FR = 2 };
      enum { WHEEL_VALID_RL = 4 };
      enum { WHEEL_VALID_RR = 8 };

    WheelTick():
      stamp(),
      wheel_tick_factor(0),
      valid_wheels(0),
      wheel_direction_fl(0),
      wheel_ticks_fl(0),
      wheel_direction_fr(0),
      wheel_ticks_fr(0),
      wheel_direction_rl(0),
      wheel_ticks_rl(0),
      wheel_direction_rr(0),
      wheel_ticks_rr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      *(outbuffer + offset + 0) = (this->wheel_tick_factor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wheel_tick_factor >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wheel_tick_factor >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wheel_tick_factor >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_tick_factor);
      *(outbuffer + offset + 0) = (this->valid_wheels >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid_wheels);
      *(outbuffer + offset + 0) = (this->wheel_direction_fl >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheel_direction_fl);
      *(outbuffer + offset + 0) = (this->wheel_ticks_fl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wheel_ticks_fl >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wheel_ticks_fl >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wheel_ticks_fl >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_ticks_fl);
      *(outbuffer + offset + 0) = (this->wheel_direction_fr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheel_direction_fr);
      *(outbuffer + offset + 0) = (this->wheel_ticks_fr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wheel_ticks_fr >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wheel_ticks_fr >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wheel_ticks_fr >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_ticks_fr);
      *(outbuffer + offset + 0) = (this->wheel_direction_rl >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheel_direction_rl);
      *(outbuffer + offset + 0) = (this->wheel_ticks_rl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wheel_ticks_rl >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wheel_ticks_rl >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wheel_ticks_rl >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_ticks_rl);
      *(outbuffer + offset + 0) = (this->wheel_direction_rr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheel_direction_rr);
      *(outbuffer + offset + 0) = (this->wheel_ticks_rr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wheel_ticks_rr >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wheel_ticks_rr >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wheel_ticks_rr >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_ticks_rr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      this->wheel_tick_factor =  ((uint32_t) (*(inbuffer + offset)));
      this->wheel_tick_factor |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_tick_factor |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wheel_tick_factor |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wheel_tick_factor);
      this->valid_wheels =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->valid_wheels);
      this->wheel_direction_fl =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->wheel_direction_fl);
      this->wheel_ticks_fl =  ((uint32_t) (*(inbuffer + offset)));
      this->wheel_ticks_fl |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_ticks_fl |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wheel_ticks_fl |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wheel_ticks_fl);
      this->wheel_direction_fr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->wheel_direction_fr);
      this->wheel_ticks_fr =  ((uint32_t) (*(inbuffer + offset)));
      this->wheel_ticks_fr |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_ticks_fr |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wheel_ticks_fr |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wheel_ticks_fr);
      this->wheel_direction_rl =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->wheel_direction_rl);
      this->wheel_ticks_rl =  ((uint32_t) (*(inbuffer + offset)));
      this->wheel_ticks_rl |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_ticks_rl |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wheel_ticks_rl |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wheel_ticks_rl);
      this->wheel_direction_rr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->wheel_direction_rr);
      this->wheel_ticks_rr =  ((uint32_t) (*(inbuffer + offset)));
      this->wheel_ticks_rr |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_ticks_rr |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->wheel_ticks_rr |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->wheel_ticks_rr);
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/WheelTick"; };
    virtual const char * getMD5() override { return "de995dcce338bea62571dd7895d4e0e2"; };

  };

}
#endif
