#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/*Include*/
#include "tim.h"

/*User difine*/


/*Variables define*/
extern uint8_t TIM1_Flag;

/*Function*/
HAL_StatusTypeDef TIM_Start(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /*__TIMER_H*/
