#include "timer_drv.h"


HAL_StatusTypeDef TIM_Start(TIM_HandleTypeDef *htim)
{
	return HAL_TIM_Base_Start_IT(htim);
}




uint8_t TIM1_Flag = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
		TIM1_Flag = 1;
    }
}
