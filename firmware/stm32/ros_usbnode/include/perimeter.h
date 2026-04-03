/****************************************************************************
* Title                 :   Perimeter wire module
* Filename              :   perimeter.h
* Author                :   Nekraus
* Origin Date           :   24/09/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file perimeter.h
*  \brief 
*
*/
#ifndef __PERIMETER_H
#define __PERIMETER_H

#ifdef OPTION_PERIMETER

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
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
typedef enum {
    COIL_LEFT = 0,
    COIL_MIDDLE = 1,
    COIL_RIGHT = 2,
    COIL_OFF = 3
}perimeter_CoilNumber_e;

/******************************************************************************
* Variables
*******************************************************************************/
extern ADC_HandleTypeDef ADC_Handle;

/******************************************************************************
* PUBLIC Function Prototypes
*******************************************************************************/
void Perimeter_vInit(void);
void Perimeter_vApp(void);
void PERIMETER_vITHandle(void);

/**
 * @brief Which signal should we listen on?
 * @param sig 0=off, 1=S1, 2=S2 (129, 130=S1,S2 plus debug output).
 */
void Perimeter_ListenOn(uint8_t sig);

/**
 * @brief Is the perimeter signal currently detected?
 */
int Perimeter_IsActive(void);

/**
 * @brief Read the current signal status of the perimeter.
 * @return There was enough data to read.
 */
int Perimeter_UpdateMsg(float *left,float *center,float *right);

/**
 * @brief Are perimeter values printed to the debug console?
 */
int Perimeter_UsesDebug(void);

#ifdef __cplusplus
}
#endif

#endif /* OPTION_PERIMETER */
#endif /*__PERIMETER_H*/ 

/*** End of File **************************************************************/
