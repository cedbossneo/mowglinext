#ifndef _ROS_mower_msgs_Status_h
#define _ROS_mower_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "mower_msgs/ESCStatus.h"

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
      typedef bool _gps_power_type;
      _gps_power_type gps_power;
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
      float ultrasonic_ranges[5];
      typedef bool _emergency_type;
      _emergency_type emergency;
      typedef float _v_charge_type;
      _v_charge_type v_charge;
      typedef float _v_battery_type;
      _v_battery_type v_battery;
      typedef float _charge_current_type;
      _charge_current_type charge_current;
      typedef bool _mow_enabled_type;
      _mow_enabled_type mow_enabled;
      typedef mower_msgs::ESCStatus _left_esc_status_type;
      _left_esc_status_type left_esc_status;
      typedef mower_msgs::ESCStatus _right_esc_status_type;
      _right_esc_status_type right_esc_status;
      typedef mower_msgs::ESCStatus _mow_esc_status_type;
      _mow_esc_status_type mow_esc_status;
      enum { MOWER_STATUS_INITIALIZING = 0 };
      enum { MOWER_STATUS_OK = 255 };

    Status():
      stamp(),
      mower_status(0),
      raspberry_pi_power(0),
      gps_power(0),
      esc_power(0),
      rain_detected(0),
      sound_module_available(0),
      sound_module_busy(0),
      ui_board_available(0),
      ultrasonic_ranges(),
      emergency(0),
      v_charge(0),
      v_battery(0),
      charge_current(0),
      mow_enabled(0),
      left_esc_status(),
      right_esc_status(),
      mow_esc_status()
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
      } u_gps_power;
      u_gps_power.real = this->gps_power;
      *(outbuffer + offset + 0) = (u_gps_power.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_power);
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
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ultrasonic_rangesi;
      u_ultrasonic_rangesi.real = this->ultrasonic_ranges[i];
      *(outbuffer + offset + 0) = (u_ultrasonic_rangesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ultrasonic_rangesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ultrasonic_rangesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ultrasonic_rangesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultrasonic_ranges[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_emergency;
      u_emergency.real = this->emergency;
      *(outbuffer + offset + 0) = (u_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency);
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
      } u_mow_enabled;
      u_mow_enabled.real = this->mow_enabled;
      *(outbuffer + offset + 0) = (u_mow_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mow_enabled);
      offset += this->left_esc_status.serialize(outbuffer + offset);
      offset += this->right_esc_status.serialize(outbuffer + offset);
      offset += this->mow_esc_status.serialize(outbuffer + offset);
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
      } u_gps_power;
      u_gps_power.base = 0;
      u_gps_power.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps_power = u_gps_power.real;
      offset += sizeof(this->gps_power);
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
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ultrasonic_rangesi;
      u_ultrasonic_rangesi.base = 0;
      u_ultrasonic_rangesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ultrasonic_rangesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ultrasonic_rangesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ultrasonic_rangesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ultrasonic_ranges[i] = u_ultrasonic_rangesi.real;
      offset += sizeof(this->ultrasonic_ranges[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_emergency;
      u_emergency.base = 0;
      u_emergency.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency = u_emergency.real;
      offset += sizeof(this->emergency);
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
      } u_mow_enabled;
      u_mow_enabled.base = 0;
      u_mow_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mow_enabled = u_mow_enabled.real;
      offset += sizeof(this->mow_enabled);
      offset += this->left_esc_status.deserialize(inbuffer + offset);
      offset += this->right_esc_status.deserialize(inbuffer + offset);
      offset += this->mow_esc_status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/Status"; };
    virtual const char * getMD5() override { return "3878b70bc9c3008ce9c808b5392f58be"; };

  };

}
#endif
