#ifndef __ULTRASONICSENSOR_H
#define __ULTRASONICSENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ULTRASONICSENSOR_Init(void);
void ULTRASONICSENSOR_App(void);
void ULTRASONICSENSOR_ReceiveIT(void);
uint32_t ULTRASONIC_MessageReceived(void);

uint32_t ULTRASONICSENSOR_u32GetLeftDistance(void);
uint32_t ULTRASONICSENSOR_u32GetRightDistance(void);

#ifdef __cplusplus
}
#endif

#endif  /* __ULTRASONICSENSOR_H */