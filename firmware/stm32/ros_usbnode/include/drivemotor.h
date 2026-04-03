/****************************************************************************
* Title                 :   drive motor module
* Filename              :   drivemotor.h
* Author                :   Nekraus
* Origin Date           :   18/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file drivemotor.h
*  \brief 
*
*/
#ifndef __DRIVEMOTOR_H
#define __DRIVEMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Includes
*******************************************************************************/

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/

/******************************************************************************
* Variables
*******************************************************************************/

extern int16_t   right_wheel_speed_val;
extern int16_t   left_wheel_speed_val;
extern uint32_t   right_encoder_ticks;  // accumulating
extern uint32_t   left_encoder_ticks;   // accumulating 
extern uint16_t  right_encoder_val;    // non accumulating 
extern uint16_t  left_encoder_val;     // non accumulating 
extern uint8_t   right_power;
extern uint8_t   left_power;
extern uint32_t  DRIVEMOTOR_u32ErrorCnt;


/******************************************************************************
* PUBLIC Function Prototypes
*******************************************************************************/

void DRIVEMOTOR_Init(void);
void DRIVEMOTOR_App_10ms(void);
void DRIVEMOTOR_App_Rx(void);
void DRIVEMOTOR_ReceiveIT(void);
void DRIVEMOTOR_SetSpeed(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir);

#ifdef __cplusplus
}
#endif
#endif /*__DRIVEMOTOR_H*/ 

/*** End of File **************************************************************/