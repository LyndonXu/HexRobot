/******************(C) copyright 天津市XXXXX有限公司 *************************
* All Rights Reserved
* 文件名：PWM.c
* 摘要: RC采集以及PWM输出相关
* 版本：0.0.1
* 作者：许龙杰
* 日期：2018年05月09日
*******************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define PWM_OUT_MAX_CHANNEL		18
#define PWM_IN_MAX_CHANNEL		10
#define PWM_OUT_PEROID			667		/* us */

typedef struct _tagStRCInput
{
	uint32_t u32RisingTime;			/* us */
	uint32_t u32FallingTime;		/* us */
	uint32_t u32Reriod;				/* us */
	uint32_t u32High;				/* us */
	uint8_t u8Mode;
}StRCInput;

enum
{
	_RC_Capture_Init = 0,
	_RC_Capture_Rising,
	_RC_Capture_Falling,	
};


static StRCInput s_stRCInput[PWM_IN_MAX_CHANNEL] = {0};


static TIM_HandleTypeDef    s_stTime3Handle;
static TIM_HandleTypeDef    s_stTime4Handle;
static TIM_HandleTypeDef    s_stTime5Handle;
static TIM_HandleTypeDef    s_stTime8Handle;
static TIM_HandleTypeDef    s_stTime9Handle;


static TIM_HandleTypeDef    s_stTime1Handle;
static TIM_HandleTypeDef    s_stTime2Handle;
static TIM_HandleTypeDef    s_stTime12Handle;


void Error_Handler(void);


static void PWMGPIOInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* timer3 ch1 & ch2 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* timer3 ch3 & ch4 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* timer 4 ch1~ch4 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	/* timer 5 ch1~ch4 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* timer 8 ch1~ch4 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/* timer 9 ch1&ch2 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* timer 1 ch1~ch4 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* timer 2 ch1 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* timer 2 ch2~ch4 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* timer 12 ch1~ch2 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void PWMOutInit(void)
{
	/* 1M, 1500Hz(Period 667) */
	uint32_t u32PrescalerValue = (uint32_t)((SystemCoreClock / 2) / 1000000) - 1;
	uint32_t u32Period = PWM_OUT_PEROID - 1;
	TIM_OC_InitTypeDef stOCConfig;

	__HAL_RCC_TIM3_CLK_ENABLE();

	s_stTime3Handle.Instance = TIM3;

	s_stTime3Handle.Init.Prescaler = u32PrescalerValue;
	s_stTime3Handle.Init.Period = u32Period;
	s_stTime3Handle.Init.ClockDivision = 0;
	s_stTime3Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_PWM_Init(&s_stTime3Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}


	stOCConfig.OCMode = TIM_OCMODE_PWM1;
	stOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	stOCConfig.OCFastMode = TIM_OCFAST_DISABLE;
	stOCConfig.Pulse = 0;
	/* Set the pulse value for channel 1 */
	{
		uint32_t i = 0;
		for (i = 0; i < 4; i++)
		{
			if(HAL_TIM_PWM_ConfigChannel(&s_stTime3Handle, &stOCConfig, (i * 4)) != HAL_OK)
			{
				/* Configuration Error */
				Error_Handler();
			}

			if(HAL_TIM_PWM_Start(&s_stTime3Handle, (i * 4)) != HAL_OK)
			{
				/* PWM Generation Error */
				Error_Handler();
			}		
		}
	}

}

void PWMInInit(void)
{
	/* 1M, 100Hz(Period 9999) */
	uint32_t u32PrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;
	TIM_IC_InitTypeDef stICConfig;

	
	/* timer 1 */
	__HAL_RCC_TIM1_CLK_ENABLE();

	s_stTime1Handle.Instance = TIM1;
	s_stTime1Handle.Init.Prescaler = u32PrescalerValue;
	s_stTime1Handle.Init.Period = 0xFFFF;
	s_stTime1Handle.Init.ClockDivision = 0;
	s_stTime1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	s_stTime1Handle.Init.RepetitionCounter = 0;

	if(HAL_TIM_IC_Init(&s_stTime1Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	
	/* Configure the Input Capture of channel */
	stICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
	stICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	stICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	stICConfig.ICFilter    = TIM_DMABURSTLENGTH_8TRANSFERS;
	{
		uint32_t i = 0;
		for (i = 0; i < 4; i++)
		{
			if(HAL_TIM_IC_ConfigChannel(&s_stTime1Handle, &stICConfig, (i * 4)) != HAL_OK)
			{
				/* Configuration Error */
				Error_Handler();
			}

			/*##-3- Start the Input Capture in interrupt mode ##########################*/
			if(HAL_TIM_IC_Start_IT(&s_stTime1Handle, (i * 4)) != HAL_OK)
			{
				/* Starting Error */
				Error_Handler();
			}		
		}
	}
	
	
	{
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 4, 0);

		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	}
	
	/* timer 2 */
	__HAL_RCC_TIM2_CLK_ENABLE();

	s_stTime2Handle.Instance = TIM2;
	s_stTime2Handle.Init.Prescaler = u32PrescalerValue / 2;
	s_stTime2Handle.Init.Period = 0xFFFF;
	s_stTime2Handle.Init.ClockDivision = 0;
	s_stTime2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	s_stTime2Handle.Init.RepetitionCounter = 0;

	if(HAL_TIM_IC_Init(&s_stTime2Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	

	/* Configure the Input Capture of channel */
	stICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
	stICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	stICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	stICConfig.ICFilter    = TIM_DMABURSTLENGTH_8TRANSFERS;
	{
		uint32_t i = 0;
		for (i = 0; i < 4; i++)
		{
			if(HAL_TIM_IC_ConfigChannel(&s_stTime2Handle, &stICConfig, (i * 4)) != HAL_OK)
			{
				/* Configuration Error */
				Error_Handler();
			}

			if(HAL_TIM_IC_Start_IT(&s_stTime2Handle, (i * 4)) != HAL_OK)
			{
				/* Starting Error */
				Error_Handler();
			}		
		}
	}
	
	
	{
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);

		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
	
	
	
	/* timer 12 */
	__HAL_RCC_TIM12_CLK_ENABLE();

	s_stTime12Handle.Instance = TIM12;
	s_stTime12Handle.Init.Prescaler = u32PrescalerValue / 2;
	s_stTime12Handle.Init.Period = 0xFFFF;
	s_stTime12Handle.Init.ClockDivision = 0;
	s_stTime12Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	s_stTime12Handle.Init.RepetitionCounter = 0;

	if(HAL_TIM_IC_Init(&s_stTime12Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	

	/* Configure the Input Capture of channel */
	stICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
	stICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	stICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	stICConfig.ICFilter    = TIM_DMABURSTLENGTH_8TRANSFERS;
	{
		uint32_t i = 0;
		for (i = 0; i < 2; i++)
		{
			if(HAL_TIM_IC_ConfigChannel(&s_stTime12Handle, &stICConfig, (i * 4)) != HAL_OK)
			{
				/* Configuration Error */
				Error_Handler();
			}

			if(HAL_TIM_IC_Start_IT(&s_stTime12Handle, (i * 4)) != HAL_OK)
			{
				/* Starting Error */
				Error_Handler();
			}		
		}
	}
	
	
	{
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 4, 0);

		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	}

}


void PWMInit(void)
{
	PWMGPIOInit();
	PWMInInit();
	PWMOutInit();
}

uint32_t * const c_pPWMOutCRRArr[PWM_OUT_MAX_CHANNEL] = 
{
	(uint32_t *)(&(TIM3->CCR1)),
	(uint32_t *)(&(TIM3->CCR2)),
	(uint32_t *)(&(TIM3->CCR3)),
	(uint32_t *)(&(TIM3->CCR4)),
	
	(uint32_t *)(&(TIM4->CCR1)),
	(uint32_t *)(&(TIM4->CCR2)),
	(uint32_t *)(&(TIM4->CCR3)),
	(uint32_t *)(&(TIM4->CCR4)),
	
	(uint32_t *)(&(TIM5->CCR1)),
	(uint32_t *)(&(TIM5->CCR2)),
	(uint32_t *)(&(TIM5->CCR3)),
	(uint32_t *)(&(TIM5->CCR4)),

	(uint32_t *)(&(TIM8->CCR1)),
	(uint32_t *)(&(TIM8->CCR2)),
	(uint32_t *)(&(TIM8->CCR3)),
	(uint32_t *)(&(TIM8->CCR4)),

	(uint32_t *)(&(TIM9->CCR1)),
	(uint32_t *)(&(TIM9->CCR2)),


};

int32_t PWMOutSetPerc(uint16_t u16Index, uint16_t u16Perc)
{
	uint16_t u16RealTime  = 0;	/*  us */
	if (u16Index >= PWM_OUT_MAX_CHANNEL)
	{
		return -1;
	}
	if (u16Perc >= 1000)
	{
		u16Perc = 1000;
	}
	
	u16RealTime = PWM_OUT_PEROID * u16Perc / 1000;
	{
		uint32_t *pCRR = c_pPWMOutCRRArr[u16Index];
		pCRR[0] = u16RealTime;
	}
		
	return u16RealTime;	
}


/* return us */
int32_t PWMInGetPerc(uint16_t u16Index)
{
	if (u16Index >= PWM_IN_MAX_CHANNEL)
	{
		return -1;
	}
	
	return (int32_t)(s_stRCInput[u16Index].u32High);	
}





/* timer1 capture handler */
void TIM1_CC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&s_stTime1Handle);
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&s_stTime2Handle);
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&s_stTime12Handle);
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *hTime)
{

	uint32_t i = 0; 
	StRCInput *pRCInput = s_stRCInput;
	TIM_IC_InitTypeDef stICConfig;
	
	if (hTime == &s_stTime1Handle)
	{
		pRCInput = s_stRCInput;
	}
	else if (hTime == &s_stTime2Handle)
	{
		pRCInput = s_stRCInput + 4;
	}
	else
	{
		pRCInput = s_stRCInput + 8;		
	}
	
	stICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
	stICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	stICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	stICConfig.ICFilter    = TIM_DMABURSTLENGTH_8TRANSFERS;

	for (i = 0; i < 4; i++)
	{
		if ((hTime->Channel & (1 << i)) != 0)
		{
			uint32_t u32Time = HAL_TIM_ReadCapturedValue(hTime, (i * 4));
			if (pRCInput->u8Mode == _RC_Capture_Init)
			{
				pRCInput->u8Mode = _RC_Capture_Falling;
				stICConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
			}
			else if (pRCInput->u8Mode == _RC_Capture_Rising)
			{
				if (u32Time < pRCInput->u32RisingTime)
				{
					pRCInput->u32Reriod = u32Time + 65536 - pRCInput->u32RisingTime;					
				}
				else
				{
					pRCInput->u32Reriod = u32Time - pRCInput->u32RisingTime;
				}
				pRCInput->u32RisingTime = u32Time;
				
				pRCInput->u8Mode = _RC_Capture_Falling;
				stICConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
				
			}
			else
			{
				if (u32Time < pRCInput->u32RisingTime)
				{
					u32Time += 65536;
				}
				pRCInput->u32High = u32Time - pRCInput->u32RisingTime;					

				pRCInput->u32FallingTime = u32Time;
				
				pRCInput->u8Mode = _RC_Capture_Rising;
				stICConfig.ICPolarity = TIM_ICPOLARITY_RISING;				
			}
			
			HAL_TIM_IC_ConfigChannel(hTime, &stICConfig, (i * 4));
			TIM_CCxChannelCmd(hTime->Instance, (i * 4), TIM_CCx_ENABLE);				
		}
		pRCInput++;
	}
}
