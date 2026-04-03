#ifndef _ROS_mower_logic_CheckPoint_h
#define _ROS_mower_logic_CheckPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_logic
{

  class CheckPoint : public ros::Msg
  {
    public:
      typedef int32_t _currentMowingPath_type;
      _currentMowingPath_type currentMowingPath;
      typedef int32_t _currentMowingArea_type;
      _currentMowingArea_type currentMowingArea;
      typedef int32_t _currentMowingPathIndex_type;
      _currentMowingPathIndex_type currentMowingPathIndex;
      typedef const char* _currentMowingPlanDigest_type;
      _currentMowingPlanDigest_type currentMowingPlanDigest;
      typedef float _currentMowingAngleIncrementSum_type;
      _currentMowingAngleIncrementSum_type currentMowingAngleIncrementSum;

    CheckPoint():
      currentMowingPath(0),
      currentMowingArea(0),
      currentMowingPathIndex(0),
      currentMowingPlanDigest(""),
      currentMowingAngleIncrementSum(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_currentMowingPath;
      u_currentMowingPath.real = this->currentMowingPath;
      *(outbuffer + offset + 0) = (u_currentMowingPath.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentMowingPath.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentMowingPath.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentMowingPath.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentMowingPath);
      union {
        int32_t real;
        uint32_t base;
      } u_currentMowingArea;
      u_currentMowingArea.real = this->currentMowingArea;
      *(outbuffer + offset + 0) = (u_currentMowingArea.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentMowingArea.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentMowingArea.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentMowingArea.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentMowingArea);
      union {
        int32_t real;
        uint32_t base;
      } u_currentMowingPathIndex;
      u_currentMowingPathIndex.real = this->currentMowingPathIndex;
      *(outbuffer + offset + 0) = (u_currentMowingPathIndex.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentMowingPathIndex.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentMowingPathIndex.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentMowingPathIndex.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentMowingPathIndex);
      uint32_t length_currentMowingPlanDigest = strlen(this->currentMowingPlanDigest);
      varToArr(outbuffer + offset, length_currentMowingPlanDigest);
      offset += 4;
      memcpy(outbuffer + offset, this->currentMowingPlanDigest, length_currentMowingPlanDigest);
      offset += length_currentMowingPlanDigest;
      union {
        float real;
        uint32_t base;
      } u_currentMowingAngleIncrementSum;
      u_currentMowingAngleIncrementSum.real = this->currentMowingAngleIncrementSum;
      *(outbuffer + offset + 0) = (u_currentMowingAngleIncrementSum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentMowingAngleIncrementSum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentMowingAngleIncrementSum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentMowingAngleIncrementSum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentMowingAngleIncrementSum);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_currentMowingPath;
      u_currentMowingPath.base = 0;
      u_currentMowingPath.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentMowingPath.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentMowingPath.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentMowingPath.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentMowingPath = u_currentMowingPath.real;
      offset += sizeof(this->currentMowingPath);
      union {
        int32_t real;
        uint32_t base;
      } u_currentMowingArea;
      u_currentMowingArea.base = 0;
      u_currentMowingArea.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentMowingArea.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentMowingArea.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentMowingArea.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentMowingArea = u_currentMowingArea.real;
      offset += sizeof(this->currentMowingArea);
      union {
        int32_t real;
        uint32_t base;
      } u_currentMowingPathIndex;
      u_currentMowingPathIndex.base = 0;
      u_currentMowingPathIndex.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentMowingPathIndex.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentMowingPathIndex.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentMowingPathIndex.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentMowingPathIndex = u_currentMowingPathIndex.real;
      offset += sizeof(this->currentMowingPathIndex);
      uint32_t length_currentMowingPlanDigest;
      arrToVar(length_currentMowingPlanDigest, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_currentMowingPlanDigest; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_currentMowingPlanDigest-1]=0;
      this->currentMowingPlanDigest = (char *)(inbuffer + offset-1);
      offset += length_currentMowingPlanDigest;
      union {
        float real;
        uint32_t base;
      } u_currentMowingAngleIncrementSum;
      u_currentMowingAngleIncrementSum.base = 0;
      u_currentMowingAngleIncrementSum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentMowingAngleIncrementSum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentMowingAngleIncrementSum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentMowingAngleIncrementSum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentMowingAngleIncrementSum = u_currentMowingAngleIncrementSum.real;
      offset += sizeof(this->currentMowingAngleIncrementSum);
     return offset;
    }

    virtual const char * getType() override { return "mower_logic/CheckPoint"; };
    virtual const char * getMD5() override { return "95cba3c345073c9090e95b74f26d0f46"; };

  };

}
#endif
