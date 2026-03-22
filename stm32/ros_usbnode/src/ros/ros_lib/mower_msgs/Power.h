#ifndef _ROS_mower_msgs_Power_h
#define _ROS_mower_msgs_Power_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace mower_msgs
{

  class Power : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _v_charge_type;
      _v_charge_type v_charge;
      typedef float _v_battery_type;
      _v_battery_type v_battery;
      typedef float _charge_current_type;
      _charge_current_type charge_current;
      typedef bool _charger_enabled_type;
      _charger_enabled_type charger_enabled;
      typedef const char* _charger_status_type;
      _charger_status_type charger_status;

    Power():
      stamp(),
      v_charge(0),
      v_battery(0),
      charge_current(0),
      charger_enabled(0),
      charger_status("")
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
        float real;
        uint32_t base;
      } u_v_charge;
      u_v_charge.real = this->v_charge;
      *(outbuffer + offset + 0) = (u_v_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_charge);
      union {
        float real;
        uint32_t base;
      } u_v_battery;
      u_v_battery.real = this->v_battery;
      *(outbuffer + offset + 0) = (u_v_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_battery);
      union {
        float real;
        uint32_t base;
      } u_charge_current;
      u_charge_current.real = this->charge_current;
      *(outbuffer + offset + 0) = (u_charge_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charge_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charge_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charge_current);
      union {
        bool real;
        uint8_t base;
      } u_charger_enabled;
      u_charger_enabled.real = this->charger_enabled;
      *(outbuffer + offset + 0) = (u_charger_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->charger_enabled);
      uint32_t length_charger_status = strlen(this->charger_status);
      varToArr(outbuffer + offset, length_charger_status);
      offset += 4;
      memcpy(outbuffer + offset, this->charger_status, length_charger_status);
      offset += length_charger_status;
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
        float real;
        uint32_t base;
      } u_v_charge;
      u_v_charge.base = 0;
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_charge = u_v_charge.real;
      offset += sizeof(this->v_charge);
      union {
        float real;
        uint32_t base;
      } u_v_battery;
      u_v_battery.base = 0;
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_battery = u_v_battery.real;
      offset += sizeof(this->v_battery);
      union {
        float real;
        uint32_t base;
      } u_charge_current;
      u_charge_current.base = 0;
      u_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charge_current = u_charge_current.real;
      offset += sizeof(this->charge_current);
      union {
        bool real;
        uint8_t base;
      } u_charger_enabled;
      u_charger_enabled.base = 0;
      u_charger_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->charger_enabled = u_charger_enabled.real;
      offset += sizeof(this->charger_enabled);
      uint32_t length_charger_status;
      arrToVar(length_charger_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_charger_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_charger_status-1]=0;
      this->charger_status = (char *)(inbuffer + offset-1);
      offset += length_charger_status;
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/Power"; };
    virtual const char * getMD5() override { return "b4d588c7a5fc639905fc448fc85b1ca9"; };

  };

}
#endif
