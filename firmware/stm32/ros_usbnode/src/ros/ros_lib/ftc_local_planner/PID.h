#ifndef _ROS_ftc_local_planner_PID_h
#define _ROS_ftc_local_planner_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace ftc_local_planner
{

  class PID : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _kp_lon_set_type;
      _kp_lon_set_type kp_lon_set;
      typedef float _kp_lat_set_type;
      _kp_lat_set_type kp_lat_set;
      typedef float _kp_ang_set_type;
      _kp_ang_set_type kp_ang_set;
      typedef float _ki_lon_set_type;
      _ki_lon_set_type ki_lon_set;
      typedef float _ki_lat_set_type;
      _ki_lat_set_type ki_lat_set;
      typedef float _ki_ang_set_type;
      _ki_ang_set_type ki_ang_set;
      typedef float _kd_lon_set_type;
      _kd_lon_set_type kd_lon_set;
      typedef float _kd_lat_set_type;
      _kd_lat_set_type kd_lat_set;
      typedef float _kd_ang_set_type;
      _kd_ang_set_type kd_ang_set;
      typedef float _lon_err_type;
      _lon_err_type lon_err;
      typedef float _lat_err_type;
      _lat_err_type lat_err;
      typedef float _ang_err_type;
      _ang_err_type ang_err;
      typedef float _ang_speed_type;
      _ang_speed_type ang_speed;
      typedef float _lin_speed_type;
      _lin_speed_type lin_speed;

    PID():
      stamp(),
      kp_lon_set(0),
      kp_lat_set(0),
      kp_ang_set(0),
      ki_lon_set(0),
      ki_lat_set(0),
      ki_ang_set(0),
      kd_lon_set(0),
      kd_lat_set(0),
      kd_ang_set(0),
      lon_err(0),
      lat_err(0),
      ang_err(0),
      ang_speed(0),
      lin_speed(0)
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
      } u_kp_lon_set;
      u_kp_lon_set.real = this->kp_lon_set;
      *(outbuffer + offset + 0) = (u_kp_lon_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp_lon_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp_lon_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp_lon_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp_lon_set);
      union {
        float real;
        uint32_t base;
      } u_kp_lat_set;
      u_kp_lat_set.real = this->kp_lat_set;
      *(outbuffer + offset + 0) = (u_kp_lat_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp_lat_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp_lat_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp_lat_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp_lat_set);
      union {
        float real;
        uint32_t base;
      } u_kp_ang_set;
      u_kp_ang_set.real = this->kp_ang_set;
      *(outbuffer + offset + 0) = (u_kp_ang_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp_ang_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp_ang_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp_ang_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp_ang_set);
      union {
        float real;
        uint32_t base;
      } u_ki_lon_set;
      u_ki_lon_set.real = this->ki_lon_set;
      *(outbuffer + offset + 0) = (u_ki_lon_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki_lon_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki_lon_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki_lon_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki_lon_set);
      union {
        float real;
        uint32_t base;
      } u_ki_lat_set;
      u_ki_lat_set.real = this->ki_lat_set;
      *(outbuffer + offset + 0) = (u_ki_lat_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki_lat_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki_lat_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki_lat_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki_lat_set);
      union {
        float real;
        uint32_t base;
      } u_ki_ang_set;
      u_ki_ang_set.real = this->ki_ang_set;
      *(outbuffer + offset + 0) = (u_ki_ang_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki_ang_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki_ang_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki_ang_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki_ang_set);
      union {
        float real;
        uint32_t base;
      } u_kd_lon_set;
      u_kd_lon_set.real = this->kd_lon_set;
      *(outbuffer + offset + 0) = (u_kd_lon_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd_lon_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd_lon_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd_lon_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd_lon_set);
      union {
        float real;
        uint32_t base;
      } u_kd_lat_set;
      u_kd_lat_set.real = this->kd_lat_set;
      *(outbuffer + offset + 0) = (u_kd_lat_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd_lat_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd_lat_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd_lat_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd_lat_set);
      union {
        float real;
        uint32_t base;
      } u_kd_ang_set;
      u_kd_ang_set.real = this->kd_ang_set;
      *(outbuffer + offset + 0) = (u_kd_ang_set.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd_ang_set.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd_ang_set.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd_ang_set.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd_ang_set);
      union {
        float real;
        uint32_t base;
      } u_lon_err;
      u_lon_err.real = this->lon_err;
      *(outbuffer + offset + 0) = (u_lon_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lon_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lon_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lon_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lon_err);
      union {
        float real;
        uint32_t base;
      } u_lat_err;
      u_lat_err.real = this->lat_err;
      *(outbuffer + offset + 0) = (u_lat_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lat_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lat_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lat_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lat_err);
      union {
        float real;
        uint32_t base;
      } u_ang_err;
      u_ang_err.real = this->ang_err;
      *(outbuffer + offset + 0) = (u_ang_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ang_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ang_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ang_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ang_err);
      union {
        float real;
        uint32_t base;
      } u_ang_speed;
      u_ang_speed.real = this->ang_speed;
      *(outbuffer + offset + 0) = (u_ang_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ang_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ang_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ang_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ang_speed);
      union {
        float real;
        uint32_t base;
      } u_lin_speed;
      u_lin_speed.real = this->lin_speed;
      *(outbuffer + offset + 0) = (u_lin_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lin_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lin_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lin_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lin_speed);
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
      } u_kp_lon_set;
      u_kp_lon_set.base = 0;
      u_kp_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp_lon_set = u_kp_lon_set.real;
      offset += sizeof(this->kp_lon_set);
      union {
        float real;
        uint32_t base;
      } u_kp_lat_set;
      u_kp_lat_set.base = 0;
      u_kp_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp_lat_set = u_kp_lat_set.real;
      offset += sizeof(this->kp_lat_set);
      union {
        float real;
        uint32_t base;
      } u_kp_ang_set;
      u_kp_ang_set.base = 0;
      u_kp_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp_ang_set = u_kp_ang_set.real;
      offset += sizeof(this->kp_ang_set);
      union {
        float real;
        uint32_t base;
      } u_ki_lon_set;
      u_ki_lon_set.base = 0;
      u_ki_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki_lon_set = u_ki_lon_set.real;
      offset += sizeof(this->ki_lon_set);
      union {
        float real;
        uint32_t base;
      } u_ki_lat_set;
      u_ki_lat_set.base = 0;
      u_ki_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki_lat_set = u_ki_lat_set.real;
      offset += sizeof(this->ki_lat_set);
      union {
        float real;
        uint32_t base;
      } u_ki_ang_set;
      u_ki_ang_set.base = 0;
      u_ki_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki_ang_set = u_ki_ang_set.real;
      offset += sizeof(this->ki_ang_set);
      union {
        float real;
        uint32_t base;
      } u_kd_lon_set;
      u_kd_lon_set.base = 0;
      u_kd_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd_lon_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd_lon_set = u_kd_lon_set.real;
      offset += sizeof(this->kd_lon_set);
      union {
        float real;
        uint32_t base;
      } u_kd_lat_set;
      u_kd_lat_set.base = 0;
      u_kd_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd_lat_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd_lat_set = u_kd_lat_set.real;
      offset += sizeof(this->kd_lat_set);
      union {
        float real;
        uint32_t base;
      } u_kd_ang_set;
      u_kd_ang_set.base = 0;
      u_kd_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd_ang_set.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd_ang_set = u_kd_ang_set.real;
      offset += sizeof(this->kd_ang_set);
      union {
        float real;
        uint32_t base;
      } u_lon_err;
      u_lon_err.base = 0;
      u_lon_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lon_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lon_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lon_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lon_err = u_lon_err.real;
      offset += sizeof(this->lon_err);
      union {
        float real;
        uint32_t base;
      } u_lat_err;
      u_lat_err.base = 0;
      u_lat_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lat_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lat_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lat_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lat_err = u_lat_err.real;
      offset += sizeof(this->lat_err);
      union {
        float real;
        uint32_t base;
      } u_ang_err;
      u_ang_err.base = 0;
      u_ang_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ang_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ang_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ang_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ang_err = u_ang_err.real;
      offset += sizeof(this->ang_err);
      union {
        float real;
        uint32_t base;
      } u_ang_speed;
      u_ang_speed.base = 0;
      u_ang_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ang_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ang_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ang_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ang_speed = u_ang_speed.real;
      offset += sizeof(this->ang_speed);
      union {
        float real;
        uint32_t base;
      } u_lin_speed;
      u_lin_speed.base = 0;
      u_lin_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lin_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lin_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lin_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lin_speed = u_lin_speed.real;
      offset += sizeof(this->lin_speed);
     return offset;
    }

    virtual const char * getType() override { return "ftc_local_planner/PID"; };
    virtual const char * getMD5() override { return "74a052c94d631fd9361aebcb670b5e39"; };

  };

}
#endif
