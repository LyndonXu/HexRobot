/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "os.h"
#include "bsp.h"
#include "os_app_hooks.h"


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define  APP_CFG_TASK_START_PRIO				16u
#define  APP_CFG_TASK_MSG_PRIO					2u
#define  APP_CFG_TASK_GUI_PRIO					4u


#define  APP_CFG_TASK_START_STK_SIZE			256u
#define  APP_CFG_TASK_MSG_STK_SIZE				256u
#define  APP_CFG_TASK_GUI_STK_SIZE				256u


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static	OS_TCB			s_stAppTaskStartTCB;
static	CPU_STK			s_stAppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];


static	OS_TCB			s_stAppTaskMsgTCB;
static	CPU_STK			s_stAppTaskMsgStk[APP_CFG_TASK_MSG_STK_SIZE];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);
static void StartOS(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

	StartOS();
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
	   clocked below the maximum system frequency, to update the voltage scaling value
	   regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	   clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
	if(HAL_GetREVID() == 0x1001)
	{
		/* Enable the Flash prefetch */
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* User may add here some code to deal with this error */
	while(1)
	{
	}
}



static  void  AppTaskStart (void *p_arg)
{
	void PWMInit(void);
	int32_t PWMOutSetPerc(uint16_t u16Index, uint16_t u16Perc);
	int32_t PWMInGetPerc(uint16_t u16Index);
	int32_t PWMOutSetRealTime(uint16_t u16Index, uint16_t u16RealTime);
	
	int32_t s32Cnt = 0;
	
    OS_ERR      err;

   (void)p_arg;
	
	OSSchedLock(&err);
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

    BSP_Init();                                                 /* Initialize BSP functions                             */

	PWMInit();
	
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif


    BSP_LED_Off(0u);
	
	

	OSSchedUnlock(&err);
	
    while (DEF_TRUE) 
	{                                          /* Task body, always written as an infinite loop.       */
		if (s32Cnt >= 25)
		{
			BSP_LED_Toggle(0u);
			s32Cnt = 0;
		}
		
#if 0
		do
		{
			int32_t s32PWMIn = PWMInGetPerc(0);
			
			if (s32PWMIn < 0)
			{
				s32PWMIn = 1500;
			}
			if (s32PWMIn < 1000)
			{
				s32PWMIn = 1000;
			}
			if (s32PWMIn > 2000)
			{
				s32PWMIn = 2000;
			}
			
			if (s32PWMIn <= 1400)
			{
				uint32_t u32Prec = s32PWMIn - 1000;	/* [0, 400] */
				u32Prec = 400 - u32Prec;
				u32Prec = u32Prec * 1000 / 400;
				PWMOutSetPerc(0, u32Prec);
				PWMOutSetPerc(1, 0);
			}
			else if (s32PWMIn >= 1600)
			{
				uint32_t u32Prec = s32PWMIn - 1600;	/* [0, 400] */
				u32Prec = u32Prec * 1000 / 400;
				PWMOutSetPerc(1, u32Prec);
				PWMOutSetPerc(0, 0);			
			}
			else
			{
				PWMOutSetPerc(0, 0);						
				PWMOutSetPerc(1, 0);
			}
			
		} while(0);
		
#endif
		
		
#if 1		
		{
			static uint32_t u32TestTime = 0;
			static bool boIsACC = false;
			static int16_t s16PWM = 1500;
			if ((HAL_GetTick() - u32TestTime) > 100)
			{
				u32TestTime = HAL_GetTick();
				if (boIsACC)
				{
					if (s16PWM >= 2400)
					{
						boIsACC = false;
						s16PWM = 2400;
					}
					else
					{
						s16PWM += 70;
					}
				}
				else
				{
					if (s16PWM <= 600)
					{
						boIsACC = true;
						s16PWM = 600;
					}
					else
					{
						s16PWM -= 70;
					}					
				}
				
				PWMOutSetRealTime(0, s16PWM);
				PWMOutSetRealTime(1, s16PWM);
				
			}		
		}
#endif
		
		
        OSTimeDlyHMSM(0u, 0u, 0u, 10u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
		
		s32Cnt++;
    }
}


static void StartOS(void)
{
	OS_ERR err = OS_ERR_NONE;

	/* STM32F4xx HAL library initialization:
	 - Configure the Flash prefetch, Flash preread and Buffer caches
	 - Systick timer is configured by default as source of time base, but user
		   can eventually implement his proper time base source (a general purpose
		   timer for example or other time source), keeping in mind that Time base
		   duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
		   handled in milliseconds basis.
	 - Low Level Initialization
   */
	HAL_Init();

	/* Configure the system clock to 168 MHz */
	SystemClock_Config();
#if 0	
	{
		#define L1	10
		#define L2	20
		#define L3	20
		
		int32_t s32Begin = SysTick->VAL;
		int32_t s32BeginI = HAL_GetTick();
		int32_t s32End;
		int32_t s32EndI;
		float f32Theta1 = 0.0f;
		float f32Theta2 = 0.0f;
		float f32Theta3 = 0.0f;

		float x = 30.0f;
		float y = 0.0f;
		float z = -10.0f;

		f32Theta3 = 0.0f - acosf((x * x + y * y + z * z - L2 * L2 - L2 * L3) / 
				(2.0f * L2 * L3));

		if (f32Theta3 < 0.0f)
		{
			//f32Theta3 = 0.0 - f32Theta3;
		}

		f32Theta2 = atanf((z) / (sqrtf(x * x + y * y))) - 
			atanf((L3 * sinf(f32Theta3)) / (L2 + L3 * cosf(f32Theta3)));

		s32End = SysTick->VAL;
		s32EndI = HAL_GetTick();
		s32End = s32Begin - s32End;/* 5.7us */
		s32Begin = s32End;
	}
#endif	
	CPU_IntDis();
	
    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    App_OS_SetAllHooks();

    OSTaskCreate(&s_stAppTaskStartTCB,                              /* Create the start task                                */
                  "App Task Start",
                  AppTaskStart,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &s_stAppTaskStartStk[0u],
                  s_stAppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    while (DEF_ON) 
	{                                            /* Should Never Get Here.                               */
        ;
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1)
	{
	}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
