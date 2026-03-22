#ifndef _ROS_xesc_msgs_XescState_h
#define _ROS_xesc_msgs_XescState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xesc_msgs
{

  class XescState : public ros::Msg
  {
    public:
      typedef uint8_t _connection_state_type;
      _connection_state_type connection_state;
      typedef uint8_t _fw_major_type;
      _fw_major_type fw_major;
      typedef uint8_t _fw_minor_type;
      _fw_minor_type fw_minor;
      typedef float _voltage_input_type;
      _voltage_input_type voltage_input;
      typedef float _temperature_pcb_type;
      _temperature_pcb_type temperature_pcb;
      typedef float _temperature_motor_type;
      _temperature_motor_type temperature_motor;
      typedef float _current_input_type;
      _current_input_type current_input;
      typedef float _duty_cycle_type;
      _duty_cycle_type duty_cycle;
      typedef uint8_t _direction_type;
      _direction_type direction;
      typedef uint32_t _tacho_type;
      _tacho_type tacho;
      typedef uint32_t _tacho_absolute_type;
      _tacho_absolute_type tacho_absolute;
      typedef int16_t _rpm_type;
      _rpm_type rpm;
      typedef uint32_t _fault_code_type;
      _fault_code_type fault_code;
      enum { XESC_FAULT_UNINITIALIZED = 1 };
      enum { XESC_FAULT_WATCHDOG = 2 };
      enum { XESC_FAULT_UNDERVOLTAGE = 4 };
      enum { XESC_FAULT_OVERVOLTAGE = 8 };
      enum { XESC_FAULT_OVERCURRENT = 16 };
      enum { XESC_FAULT_OVERTEMP_MOTOR = 32 };
      enum { XESC_FAULT_OVERTEMP_PCB = 64 };
      enum { XESC_FAULT_INVALID_HALL = 128 };
      enum { XESC_FAULT_INTERNAL_ERROR = 256 };
      enum { XESC_FAULT_OPEN_LOAD = 512 };
      enum { XESC_CONNECTION_STATE_DISCONNECTED = 0 };
      enum { XESC_CONNECTION_STATE_WAITING_FOR_FW = 1 };
      enum { XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW = 2 };
      enum { XESC_CONNECTION_STATE_CONNECTED = 3 };

    XescState():
      connection_state(0),
      fw_major(0),
      fw_minor(0),
      voltage_input(0),
      temperature_pcb(0),
      temperature_motor(0),
      current_input(0),
      duty_cycle(0),
      direction(0),
      tacho(0),
      tacho_absolute(0),
      rpm(0),
      fault_code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->connection_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->connection_state);
      *(outbuffer + offset + 0) = (this->fw_major >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fw_major);
      *(outbuffer + offset + 0) = (this->fw_minor >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fw_minor);
      offset += serializeAvrFloat64(outbuffer + offset, this->voltage_input);
      offset += serializeAvrFloat64(outbuffer + offset, this->temperature_pcb);
      offset += serializeAvrFloat64(outbuffer + offset, this->temperature_motor);
      offset += serializeAvrFloat64(outbuffer + offset, this->current_input);
      offset += serializeAvrFloat64(outbuffer + offset, this->duty_cycle);
      *(outbuffer + offset + 0) = (this->direction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction);
      *(outbuffer + offset + 0) = (this->tacho >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tacho >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tacho >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tacho >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tacho);
      *(outbuffer + offset + 0) = (this->tacho_absolute >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tacho_absolute >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tacho_absolute >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tacho_absolute >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tacho_absolute);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm;
      u_rpm.real = this->rpm;
      *(outbuffer + offset + 0) = (u_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm);
      *(outbuffer + offset + 0) = (this->fault_code >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fault_code >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fault_code >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fault_code >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fault_code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->connection_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->connection_state);
      this->fw_major =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fw_major);
      this->fw_minor =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fw_minor);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->voltage_input));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->temperature_pcb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->temperature_motor));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_input));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->duty_cycle));
      this->direction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->direction);
      this->tacho =  ((uint32_t) (*(inbuffer + offset)));
      this->tacho |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tacho |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tacho |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tacho);
      this->tacho_absolute =  ((uint32_t) (*(inbuffer + offset)));
      this->tacho_absolute |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tacho_absolute |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tacho_absolute |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tacho_absolute);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm;
      u_rpm.base = 0;
      u_rpm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm = u_rpm.real;
      offset += sizeof(this->rpm);
      this->fault_code =  ((uint32_t) (*(inbuffer + offset)));
      this->fault_code |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fault_code |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fault_code |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fault_code);
     return offset;
    }

    virtual const char * getType() override { return "xesc_msgs/XescState"; };
    virtual const char * getMD5() override { return "e4c9b49bab3a68c3798616a60f809abe"; };

  };

}
#endif
