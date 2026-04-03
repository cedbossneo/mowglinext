#ifndef _ROS_mower_msgs_Emergency_h
#define _ROS_mower_msgs_Emergency_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace mower_msgs
{

  class Emergency : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef bool _active_emergency_type;
      _active_emergency_type active_emergency;
      typedef bool _latched_emergency_type;
      _latched_emergency_type latched_emergency;
      typedef const char* _reason_type;
      _reason_type reason;

    Emergency():
      stamp(),
      active_emergency(0),
      latched_emergency(0),
      reason("")
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
      union {
        bool real;
        uint8_t base;
      } u_active_emergency;
      u_active_emergency.real = this->active_emergency;
      *(outbuffer + offset + 0) = (u_active_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active_emergency);
      union {
        bool real;
        uint8_t base;
      } u_latched_emergency;
      u_latched_emergency.real = this->latched_emergency;
      *(outbuffer + offset + 0) = (u_latched_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->latched_emergency);
      uint32_t length_reason = strlen(this->reason);
      varToArr(outbuffer + offset, length_reason);
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
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
      union {
        bool real;
        uint8_t base;
      } u_active_emergency;
      u_active_emergency.base = 0;
      u_active_emergency.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active_emergency = u_active_emergency.real;
      offset += sizeof(this->active_emergency);
      union {
        bool real;
        uint8_t base;
      } u_latched_emergency;
      u_latched_emergency.base = 0;
      u_latched_emergency.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->latched_emergency = u_latched_emergency.real;
      offset += sizeof(this->latched_emergency);
      uint32_t length_reason;
      arrToVar(length_reason, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/Emergency"; };
    virtual const char * getMD5() override { return "d7eccf6a6a9c8fdeb4675252bc74b404"; };

  };

}
#endif
