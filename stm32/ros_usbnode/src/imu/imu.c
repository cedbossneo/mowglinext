
/**
  ******************************************************************************
  * @file    imu.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Mowgli IMU calibration routines as long as they are IMU independent 
  ******************************************************************************
  * @attention
  *
  * details: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration?view=all
  *          https://makersportal.com/blog/calibration-of-a-magnetometer-with-raspberry-pi
  ******************************************************************************
  */

#include <math.h>

#include "imu/imu.h"
#include "imu/lsm6.h"
#include "imu/mpu6050.h"
#include "imu/wt901.h"
#include "imu/icm45686.h"
#include "imu/lis3mdl.h"
#include "i2c.h"
#include "main.h"

IMU_ReadAccelerometerRaw imuReadAccelerometerRaw=NULL;
IMU_ReadGyroRaw imuReadGyroRaw=NULL;
IMU_ReadGyroRaw imuReadMagRaw=NULL;  /* reuse same typedef: void(*)(float*,float*,float*) */

/* accelerometer calibration values */
float imu_cal_ax = 0.0;
float imu_cal_ay = 0.0;
float imu_cal_az = 0.0;
/* gyro calibration values */
float imu_cal_gx = 0.0;
float imu_cal_gy = 0.0;
float imu_cal_gz = 0.0;
/* onboard accelerometer calibration values */
float onboard_imu_cal_ax = 0.0;
float onboard_imu_cal_ay = 0.0;
float onboard_imu_cal_az = 0.0;

/* covariance matrixes for IMU data */
float imu_cov_ax = 0.01;
float imu_cov_ay = 0.01;
float imu_cov_az = 0.01;
// ---------------------
float imu_cov_gx = 0.1;
float imu_cov_gy = 0.1;
float imu_cov_gz = 0.1;
// ---------------------
float onboard_imu_cov_ax = 0.01;
float onboard_imu_cov_ay = 0.01;
float onboard_imu_cov_az = 0.01;
// ---------------------

static int assertAccelerometer() {
  return debug_assert(imuReadAccelerometerRaw!=NULL,"Usage of non installed accelerometer\r\n");
}

static int assertGyro() {
  return debug_assert(imuReadGyroRaw!=NULL,"Usage of non installed gryometer\r\n");
}

int IMU_HasAccelerometer() {
  return imuReadAccelerometerRaw!=NULL;
}

int IMU_HasGyro() {
  return imuReadGyroRaw!=NULL;
}

/**
  * @brief  Reads the 3 accelerometer axis and stores them in *x,*y,*z  
  * 
  * units are m/s^2 uncalibrated
  */ 
void IMU_ReadAccelerometer(float *x, float *y, float *z)
{
  if (assertAccelerometer()) return;
    float imu_x, imu_y, imu_z;
    imuReadAccelerometerRaw(&imu_x, &imu_y, &imu_z);
    // apply calibration
    *x = imu_x - imu_cal_ax;
    *y = imu_y - imu_cal_ay;
    *z = imu_z - imu_cal_az;
}

/*
 * Set covariance matrix values
 */
void IMU_AccelerometerSetCovariance(float *cm)
{
   cm[0] = imu_cov_ax;
   cm[4] = imu_cov_ay;
   cm[8] = imu_cov_az;
}


/**
  * @brief  Reads the 3 accelerometer gyro and stores them in *x,*y,*z  
  * 
  * units are rad/sec uncalibrated
  */ 
void IMU_ReadGyro(float *x, float *y, float *z)
{
  if (assertGyro()) return;
  float imu_x, imu_y, imu_z;
  imuReadGyroRaw(&imu_x, &imu_y, &imu_z);
  // apply calibration
  *x = imu_x - imu_cal_gx;
  *y = imu_y - imu_cal_gy;
  *z = imu_z - imu_cal_gz;
}

/**
  * @brief  Reads the 3 magnetometer channels (if available)
  * units are uT (microtesla), uncalibrated
  */
void IMU_ReadMag(float *x, float *y, float *z)
{
  if (imuReadMagRaw != NULL) {
    imuReadMagRaw(x, y, z);
  } else {
    *x = 0.0f;
    *y = 0.0f;
    *z = 0.0f;
  }
}

int IMU_HasMag(void)
{
  return imuReadMagRaw != NULL;
}

/*
 * Set covariance matrix values
 */
void IMU_GyroSetCovariance(float *cm)
{
   cm[0] = imu_cov_gx;
   cm[4] = imu_cov_gy;
   cm[8] = imu_cov_gz;
}

/*
 * Read onboard IMU acceleration in ms^2
 */
void IMU_Onboard_ReadAccelerometer(float *x, float *y, float *z)
{
   I2C_ReadAccelerometer(x, y, z);
}

/*
 * Set covariance matrix values
 */
void IMU_Onboard_AccelerometerSetCovariance(float *cm)
{
   cm[0] = onboard_imu_cov_ax;
   cm[4] = onboard_imu_cov_ay;
   cm[8] = onboard_imu_cov_az;
}

/*
 * Read onboard IMU temperature in °C
 */
float IMU_Onboard_ReadTemp(void)
{
  return(I2C_ReadAccelerometerTemp());
}


/**
  * @brief Calibrates IMU accelerometers and gyro by averaging and storing those values as calibration factors 
  * it expects that the bot is leveled and not moving
  * 
  */ 
void IMU_CalibrateExternal()
{
    float imu_sample_x[IMU_CAL_SAMPLES], imu_sample_y[IMU_CAL_SAMPLES], imu_sample_z[IMU_CAL_SAMPLES]; 
    float stddev_x, stddev_y, stddev_z;
    float mean_x, mean_y, mean_z;    
    float sum_x, sum_y, sum_z;
    uint16_t i;
    
    
    debug_printf("    > External IMU Calibration started - make sure bot is level and standing still ...\r\n");    
    
    /************************************/
    /* calibrate external accelerometer */
    /************************************/
    if (imuReadAccelerometerRaw) {
      sum_x = sum_y = sum_z = 0;
      for (i=0; i<IMU_CAL_SAMPLES; i++)
      {
        imuReadAccelerometerRaw(&imu_sample_x[i], &imu_sample_y[i], &imu_sample_z[i]);
        sum_x += imu_sample_x[i];
        sum_y += imu_sample_y[i];
        sum_z += imu_sample_z[i];
        HAL_Delay(10);      
      }
      mean_x = sum_x / IMU_CAL_SAMPLES;
      mean_y = sum_y / IMU_CAL_SAMPLES;
      mean_z = sum_z / IMU_CAL_SAMPLES;
      imu_cal_ax = mean_x;
      imu_cal_ay = mean_y;
      imu_cal_az = 0;    // we dont want to calibrate Z because our IMU Sensor fusion stack expects gravity
      debug_printf("    > External IMU Calibration factors accelerometer [%f %f %f]\r\n", imu_cal_ax, imu_cal_ay, imu_cal_az);
      stddev_x = stddev_y = stddev_z = 0;
      for (i=0; i<IMU_CAL_SAMPLES; i++)
      {
          stddev_x += pow(imu_sample_x[i] - mean_x, 2);
          stddev_y += pow(imu_sample_y[i] - mean_y, 2);
          stddev_z += pow(imu_sample_z[i] - mean_z, 2);        
      }
      imu_cov_ax = stddev_x / IMU_CAL_SAMPLES;
      imu_cov_ay = stddev_y / IMU_CAL_SAMPLES;
      imu_cov_az = stddev_z / IMU_CAL_SAMPLES;
      debug_printf("    > External IMU Calibration accelerometer covariance diagonal [%f %f %f]\r\n", imu_cov_ax, imu_cov_ay, imu_cov_az); 
    }

    /***************************/
    /* calibrate external gyro */
    /***************************/
    if (imuReadGyroRaw) {
      sum_x = sum_y = sum_z = 0;    
      for (i=0; i<IMU_CAL_SAMPLES; i++)
      {
        imuReadGyroRaw(&imu_sample_x[i], &imu_sample_y[i], &imu_sample_z[i]);
        sum_x += imu_sample_x[i];
        sum_y += imu_sample_y[i];
        sum_z += imu_sample_z[i];
        HAL_Delay(10);      
      }
      mean_x = sum_x / IMU_CAL_SAMPLES;
      mean_y = sum_y / IMU_CAL_SAMPLES;
      mean_z = sum_z / IMU_CAL_SAMPLES;
      imu_cal_gx = mean_x;
      imu_cal_gy = mean_y;
      imu_cal_gz = mean_z;
      debug_printf("    > External IMU Calibration factors gyro [%f %f %f]\r\n", imu_cal_gx, imu_cal_gy, imu_cal_gz);
      stddev_x = stddev_y = stddev_z = 0;
      for (i=0; i<IMU_CAL_SAMPLES; i++)
      {
        stddev_x += pow(imu_sample_x[i] - mean_x, 2);
        stddev_y += pow(imu_sample_y[i] - mean_y, 2);
        stddev_z += pow(imu_sample_z[i] - mean_z, 2);        
      }
      imu_cov_gx = stddev_x / IMU_CAL_SAMPLES;
      imu_cov_gy = stddev_y / IMU_CAL_SAMPLES;
      imu_cov_gz = stddev_z / IMU_CAL_SAMPLES;
      debug_printf("    > External IMU Calibration gyro covariance diagonal [%f %f %f]\r\n", imu_cov_gx, imu_cov_gy, imu_cov_gz); 
    }
}

void IMU_CalibrateOnboard()
{
    float imu_sample_x[IMU_CAL_SAMPLES], imu_sample_y[IMU_CAL_SAMPLES], imu_sample_z[IMU_CAL_SAMPLES]; 
    float stddev_x, stddev_y, stddev_z;
    float mean_x, mean_y, mean_z;    
    float sum_x, sum_y, sum_z;
    uint16_t i;

    debug_printf("    > Onboard IMU Calibration started - make sure bot is level and standing still ...\r\n");  

    /************************************/
    /* calibrate onboard accelerometer  */
    /************************************/
    sum_x = sum_y = sum_z = 0;    
    for (i=0; i<IMU_CAL_SAMPLES; i++)
    {
      I2C_ReadAccelerometer(&imu_sample_x[i], &imu_sample_y[i], &imu_sample_z[i]);
      sum_x += imu_sample_x[i];
      sum_y += imu_sample_y[i];
      sum_z += imu_sample_z[i];
      HAL_Delay(10);      
    }
    mean_x = sum_x / IMU_CAL_SAMPLES;
    mean_y = sum_y / IMU_CAL_SAMPLES;
    mean_z = sum_z / IMU_CAL_SAMPLES;
    onboard_imu_cal_ax = mean_x;
    onboard_imu_cal_ay = mean_y;
    onboard_imu_cal_az = 0;    // we dont want to calibrate Z because our IMU Sensor fusion stack expects gravity
    debug_printf("    > Onboard IMU Calibration factors accelerometer [%f %f %f]\r\n", onboard_imu_cal_ax, onboard_imu_cal_ay, onboard_imu_cal_az);
    stddev_x = stddev_y = stddev_z = 0;
    for (i=0; i<IMU_CAL_SAMPLES; i++)
    {
        stddev_x += pow(imu_sample_x[i] - mean_x, 2);
        stddev_y += pow(imu_sample_y[i] - mean_y, 2);
        stddev_z += pow(imu_sample_z[i] - mean_z, 2);        
    }
    onboard_imu_cov_ax = stddev_x / IMU_CAL_SAMPLES;
    onboard_imu_cov_ay = stddev_y / IMU_CAL_SAMPLES;
    onboard_imu_cov_az = stddev_z / IMU_CAL_SAMPLES;
    debug_printf("    > Onboard IMU Calibration accelerometer covariance diagonal [%f %f %f]\r\n", onboard_imu_cov_ax, onboard_imu_cov_ay, onboard_imu_cov_az); 
}

void IMU_Normalize( VECTOR* p )
{
    double w = sqrt( p->x * p->x + p->y * p->y + p->z * p->z );
    p->x /= w;
    p->y /= w;
    p->z /= w;
}

void IMU_Init() {
  imuReadAccelerometerRaw=NULL;
  imuReadGyroRaw=NULL;
  imuReadMagRaw=NULL;

  uint32_t l_u32Timestamp = HAL_GetTick();
while (imuReadAccelerometerRaw == NULL && ((HAL_GetTick() - l_u32Timestamp) < 20000) )
{
  #ifndef DISABLE_LSM6
    if (LSM6_TestDevice()) {
      LSM6_Init();
      imuReadAccelerometerRaw=LSM6_ReadAccelerometerRaw;
      imuReadGyroRaw=LSM6_ReadGyroRaw;
    }
  #endif

  #ifndef DISABLE_WT901
    if ((!imuReadGyroRaw || !imuReadAccelerometerRaw) && WT901_TestDevice()) {
      WT901_Init();
      imuReadAccelerometerRaw=WT901_ReadAccelerometerRaw;
      imuReadGyroRaw=WT901_ReadGyroRaw;
      imuReadMagRaw=WT901_ReadMagRaw;
    }
  #endif

  #ifndef DISABLE_MPU6050
    if ((!imuReadGyroRaw || !imuReadAccelerometerRaw) && MPU6050_TestDevice()) {
      MPU6050_Init();
      imuReadAccelerometerRaw=MPU6050_ReadAccelerometerRaw;
      imuReadGyroRaw=MPU6050_ReadGyroRaw;
    }
  #endif

  HAL_Delay(20);
}

if(imuReadAccelerometerRaw == NULL){
  chirp(10);
}

#ifndef DISABLE_ICM45686
  if ((!imuReadGyroRaw || !imuReadAccelerometerRaw) && ICM45686_TestDevice()) {
    ICM45686_Init();
    imuReadAccelerometerRaw=ICM45686_ReadAccelerometerRaw;
    imuReadGyroRaw=ICM45686_ReadGyroRaw;
  }
#endif

  /* Magnetometer: try LIS3MDL (separate chip on Pololu boards) if no
   * mag was set by the accel/gyro IMU driver (e.g. WT901 has built-in mag,
   * but LSM6/MPU6050/ICM45686 do not). */
#ifndef DISABLE_LIS3MDL
  if (imuReadMagRaw == NULL && LIS3MDL_TestDevice()) {
    LIS3MDL_Init();
    imuReadMagRaw = LIS3MDL_ReadMagRaw;
  }
#endif

}