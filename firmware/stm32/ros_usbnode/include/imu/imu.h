
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f_board_hal.h"

typedef struct 
{
    double x, y, z;
} VECTOR;

/*
 * conversions to ROS units
 */
#define RAD_PER_G               0.01745             // convert °/sec to rad/sec
#define MS2_PER_G               9.80665             // convert g to m/s^2 
#define T_PER_GAUSS             1/10000             // convert Gauss to T

/*
 * IMU functions that a compatible IMU needs to be able to provide
 */

void IMU_ReadAccelerometer(float *x, float *y, float *z);
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z);
float IMU_Onboard_ReadTemp(void);
void IMU_ReadGyro(float *x, float *y, float *z);
typedef float (*IMU_ReadBarometerTemperatureC)(void);
typedef float (*IMU_ReadBarometerAltitudeMeters)(void);
void IMU_Onboard_AccelerometerSetCovariance(float *cm);
void IMU_AccelerometerSetCovariance(float *cm);
void IMU_GyroSetCovariance(float *cm);
void IMU_Normalize( VECTOR* p );

/* Any external IMU needs to implement the following functions and adhere to the ROS REP 103 standard (https://www.ros.org/reps/rep-0103.html) */
typedef void (*IMU_ReadGyroRaw)(float *x, float *y, float *z);
typedef void (*IMU_ReadAccelerometerRaw)(float *x, float *y, float *z);
/* end of functions to implement for IMU */

void IMU_Init();
int IMU_HasAccelerometer();
int IMU_HasGyro();
int IMU_HasMag();
void IMU_ReadMag(float *x, float *y, float *z);

/* IMU calibration (accel/gyro only) */
#define IMU_CAL_SAMPLES     100
void IMU_CalibrateExternal(void);
void IMU_CalibrateOnboard(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
