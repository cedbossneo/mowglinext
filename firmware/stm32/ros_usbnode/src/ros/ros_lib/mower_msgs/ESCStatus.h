#ifndef _ROS_mower_msgs_ESCStatus_h
#define _ROS_mower_msgs_ESCStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

  class ESCStatus : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      typedef float _current_type;
      _current_type current;
      typedef uint32_t _tacho_type;
      _tacho_type tacho;
      typedef int16_t _rpm_type;
      _rpm_type rpm;
      typedef float _temperature_motor_type;
      _temperature_motor_type temperature_motor;
      typedef float _temperature_pcb_type;
      _temperature_pcb_type temperature_pcb;
      enum { ESC_STATUS_DISCONNECTED = 99 };
      enum { ESC_STATUS_ERROR = 100 };
      enum { ESC_STATUS_STALLED = 150 };
      enum { ESC_STATUS_OK = 200 };
      enum { ESC_STATUS_RUNNING = 201 };

    ESCStatus():
      status(0),
      current(0),
      tacho(0),
      rpm(0),
      temperature_motor(0),
      temperature_pcb(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      *(outbuffer + offset + 0) = (this->tacho >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tacho >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tacho >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tacho >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tacho);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm;
      u_rpm.real = this->rpm;
      *(outbuffer + offset + 0) = (u_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm);
      union {
        float real;
        uint32_t base;
      } u_temperature_motor;
      u_temperature_motor.real = this->temperature_motor;
      *(outbuffer + offset + 0) = (u_temperature_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature_motor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature_motor);
      union {
        float real;
        uint32_t base;
      } u_temperature_pcb;
      u_temperature_pcb.real = this->temperature_pcb;
      *(outbuffer + offset + 0) = (u_temperature_pcb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature_pcb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature_pcb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature_pcb.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature_pcb);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      this->tacho =  ((uint32_t) (*(inbuffer + offset)));
      this->tacho |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tacho |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tacho |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tacho);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm;
      u_rpm.base = 0;
      u_rpm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm = u_rpm.real;
      offset += sizeof(this->rpm);
      union {
        float real;
        uint32_t base;
      } u_temperature_motor;
      u_temperature_motor.base = 0;
      u_temperature_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature_motor = u_temperature_motor.real;
      offset += sizeof(this->temperature_motor);
      union {
        float real;
        uint32_t base;
      } u_temperature_pcb;
      u_temperature_pcb.base = 0;
      u_temperature_pcb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature_pcb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature_pcb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature_pcb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature_pcb = u_temperature_pcb.real;
      offset += sizeof(this->temperature_pcb);
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/ESCStatus"; };
    virtual const char * getMD5() override { return "dbda697be744eb1134b10982a99ce3ed"; };

  };

}
#endif
