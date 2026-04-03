#ifndef _ROS_mower_msgs_Status_h
#define _ROS_mower_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace mower_msgs
{

  class Status : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef uint8_t _mower_status_type;
      _mower_status_type mower_status;
      typedef bool _raspberry_pi_power_type;
      _raspberry_pi_power_type raspberry_pi_power;
      typedef bool _is_charging_type;
      _is_charging_type is_charging;
      typedef bool _esc_power_type;
      _esc_power_type esc_power;
      typedef bool _rain_detected_type;
      _rain_detected_type rain_detected;
      typedef bool _sound_module_available_type;
      _sound_module_available_type sound_module_available;
      typedef bool _sound_module_busy_type;
      _sound_module_busy_type sound_module_busy;
      typedef bool _ui_board_available_type;
      _ui_board_available_type ui_board_available;
      typedef bool _mow_enabled_type;
      _mow_enabled_type mow_enabled;
      typedef uint8_t _mower_esc_status_type;
      _mower_esc_status_type mower_esc_status;
      typedef float _mower_esc_temperature_type;
      _mower_esc_temperature_type mower_esc_temperature;
      typedef float _mower_esc_current_type;
      _mower_esc_current_type mower_esc_current;
      typedef float _mower_motor_temperature_type;
      _mower_motor_temperature_type mower_motor_temperature;
      typedef float _mower_motor_rpm_type;
      _mower_motor_rpm_type mower_motor_rpm;
      enum { MOWER_STATUS_INITIALIZING = 0 };
      enum { MOWER_STATUS_OK = 255 };

    Status():
      stamp(),
      mower_status(0),
      raspberry_pi_power(0),
      is_charging(0),
      esc_power(0),
      rain_detected(0),
      sound_module_available(0),
      sound_module_busy(0),
      ui_board_available(0),
      mow_enabled(0),
      mower_esc_status(0),
      mower_esc_temperature(0),
      mower_esc_current(0),
      mower_motor_temperature(0),
      mower_motor_rpm(0)
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
      *(outbuffer + offset + 0) = (this->mower_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mower_status);
      union {
        bool real;
        uint8_t base;
      } u_raspberry_pi_power;
      u_raspberry_pi_power.real = this->raspberry_pi_power;
      *(outbuffer + offset + 0) = (u_raspberry_pi_power.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->raspberry_pi_power);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.real = this->is_charging;
      *(outbuffer + offset + 0) = (u_is_charging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_charging);
      union {
        bool real;
        uint8_t base;
      } u_esc_power;
      u_esc_power.real = this->esc_power;
      *(outbuffer + offset + 0) = (u_esc_power.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->esc_power);
      union {
        bool real;
        uint8_t base;
      } u_rain_detected;
      u_rain_detected.real = this->rain_detected;
      *(outbuffer + offset + 0) = (u_rain_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rain_detected);
      union {
        bool real;
        uint8_t base;
      } u_sound_module_available;
      u_sound_module_available.real = this->sound_module_available;
      *(outbuffer + offset + 0) = (u_sound_module_available.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sound_module_available);
      union {
        bool real;
        uint8_t base;
      } u_sound_module_busy;
      u_sound_module_busy.real = this->sound_module_busy;
      *(outbuffer + offset + 0) = (u_sound_module_busy.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sound_module_busy);
      union {
        bool real;
        uint8_t base;
      } u_ui_board_available;
      u_ui_board_available.real = this->ui_board_available;
      *(outbuffer + offset + 0) = (u_ui_board_available.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ui_board_available);
      union {
        bool real;
        uint8_t base;
      } u_mow_enabled;
      u_mow_enabled.real = this->mow_enabled;
      *(outbuffer + offset + 0) = (u_mow_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mow_enabled);
      *(outbuffer + offset + 0) = (this->mower_esc_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mower_esc_status);
      union {
        float real;
        uint32_t base;
      } u_mower_esc_temperature;
      u_mower_esc_temperature.real = this->mower_esc_temperature;
      *(outbuffer + offset + 0) = (u_mower_esc_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mower_esc_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mower_esc_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mower_esc_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mower_esc_temperature);
      union {
        float real;
        uint32_t base;
      } u_mower_esc_current;
      u_mower_esc_current.real = this->mower_esc_current;
      *(outbuffer + offset + 0) = (u_mower_esc_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mower_esc_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mower_esc_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mower_esc_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mower_esc_current);
      union {
        float real;
        uint32_t base;
      } u_mower_motor_temperature;
      u_mower_motor_temperature.real = this->mower_motor_temperature;
      *(outbuffer + offset + 0) = (u_mower_motor_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mower_motor_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mower_motor_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mower_motor_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mower_motor_temperature);
      union {
        float real;
        uint32_t base;
      } u_mower_motor_rpm;
      u_mower_motor_rpm.real = this->mower_motor_rpm;
      *(outbuffer + offset + 0) = (u_mower_motor_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mower_motor_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mower_motor_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mower_motor_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mower_motor_rpm);
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
      this->mower_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mower_status);
      union {
        bool real;
        uint8_t base;
      } u_raspberry_pi_power;
      u_raspberry_pi_power.base = 0;
      u_raspberry_pi_power.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->raspberry_pi_power = u_raspberry_pi_power.real;
      offset += sizeof(this->raspberry_pi_power);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.base = 0;
      u_is_charging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_charging = u_is_charging.real;
      offset += sizeof(this->is_charging);
      union {
        bool real;
        uint8_t base;
      } u_esc_power;
      u_esc_power.base = 0;
      u_esc_power.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->esc_power = u_esc_power.real;
      offset += sizeof(this->esc_power);
      union {
        bool real;
        uint8_t base;
      } u_rain_detected;
      u_rain_detected.base = 0;
      u_rain_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rain_detected = u_rain_detected.real;
      offset += sizeof(this->rain_detected);
      union {
        bool real;
        uint8_t base;
      } u_sound_module_available;
      u_sound_module_available.base = 0;
      u_sound_module_available.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sound_module_available = u_sound_module_available.real;
      offset += sizeof(this->sound_module_available);
      union {
        bool real;
        uint8_t base;
      } u_sound_module_busy;
      u_sound_module_busy.base = 0;
      u_sound_module_busy.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sound_module_busy = u_sound_module_busy.real;
      offset += sizeof(this->sound_module_busy);
      union {
        bool real;
        uint8_t base;
      } u_ui_board_available;
      u_ui_board_available.base = 0;
      u_ui_board_available.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ui_board_available = u_ui_board_available.real;
      offset += sizeof(this->ui_board_available);
      union {
        bool real;
        uint8_t base;
      } u_mow_enabled;
      u_mow_enabled.base = 0;
      u_mow_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mow_enabled = u_mow_enabled.real;
      offset += sizeof(this->mow_enabled);
      this->mower_esc_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mower_esc_status);
      union {
        float real;
        uint32_t base;
      } u_mower_esc_temperature;
      u_mower_esc_temperature.base = 0;
      u_mower_esc_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mower_esc_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mower_esc_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mower_esc_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mower_esc_temperature = u_mower_esc_temperature.real;
      offset += sizeof(this->mower_esc_temperature);
      union {
        float real;
        uint32_t base;
      } u_mower_esc_current;
      u_mower_esc_current.base = 0;
      u_mower_esc_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mower_esc_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mower_esc_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mower_esc_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mower_esc_current = u_mower_esc_current.real;
      offset += sizeof(this->mower_esc_current);
      union {
        float real;
        uint32_t base;
      } u_mower_motor_temperature;
      u_mower_motor_temperature.base = 0;
      u_mower_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mower_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mower_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mower_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mower_motor_temperature = u_mower_motor_temperature.real;
      offset += sizeof(this->mower_motor_temperature);
      union {
        float real;
        uint32_t base;
      } u_mower_motor_rpm;
      u_mower_motor_rpm.base = 0;
      u_mower_motor_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mower_motor_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mower_motor_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mower_motor_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mower_motor_rpm = u_mower_motor_rpm.real;
      offset += sizeof(this->mower_motor_rpm);
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/Status"; };
    virtual const char * getMD5() override { return "5b038ed6144dd306d00a868172f69576"; };

  };

}
#endif
