/**
******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
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
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
TIM_HandleTypeDef Tim3_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 


char lcd_buffer[6];    // LCD display buffer

//param keys
int speed=5;
int spin=0;
int full=1;

//control keys
int left=0;
int right=0; 
int up=0; 
int down=0; 
int select=0;

//pin control keys
int full_pin=0;
int half_pin=0;

//next stator
int move=0;

//pin rep
uint16_t pin[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};



/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void TIM3_OC_Config(void);
static void Tim3_Config(void);
static void updateSpeed(void);
static void applySpeed(void);
static void updateSpin(void);
static void GPIO_Config(void);
static void initSpin(void);
static void spinHalf(void);
static void spinFull(void);





//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	HAL_Init();
	

	SystemClock_Config();   //sysclock is 80Hz. HClkm apb1 an apb2 are all 80Mhz.
  
	HAL_InitTick(0x0000); // set systick's priority to the highest.

	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);


	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);  
	Tim3_Config();
	TIM3_OC_Config();
	GPIO_Config();
	



	
	while (1)
	{
		//half and full with select button 
		//speed up and down with up and down keys
		//CCW and CW with right and left keys 
		if (up!=0 || down!=0){
			updateSpeed();
			applySpeed();
		}
		if(right==1){
			right=0;
			spin=1;
			updateSpin();
		}
		if (left==1){
			left=0;
			spin=0;
			updateSpin();
		}
		if (select==1){
			select=0;
			full=(full+1)%2;
			initSpin();

		}
		if (move==1)
		{
			move=0;
			if (full==1)
			{//full
				spinFull();
			}
			else
			{//half
				spinHalf();
			}
		}
		
		
		

	} //end of while 1

}

void initSpin(void)
{	
	half_pin=full_pin;
	int i =0; 
	for (i=0; i < 4; i++){
		HAL_GPIO_WritePin(GPIOE, pin[i], GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(GPIOE, pin[full_pin], GPIO_PIN_SET);

	
}



void spinFull(void)
{
	HAL_GPIO_WritePin(GPIOE, pin[full_pin], GPIO_PIN_RESET);
	full_pin=(full_pin+1)%4;
	HAL_GPIO_WritePin(GPIOE, pin[full_pin], GPIO_PIN_SET);
	BSP_LED_Toggle(LED5);
}

void spinHalf(void)
{
	if(full_pin!=half_pin)
	{
		//turn off old
		HAL_GPIO_WritePin(GPIOE, pin[full_pin], GPIO_PIN_RESET);
		full_pin=half_pin;
	}		
	else 
	{
		//start new
		half_pin=(full_pin+1)%4;
		HAL_GPIO_WritePin(GPIOE, pin[half_pin], GPIO_PIN_SET);
	}
	
}

void updateSpin(void)
{
	initSpin();
	if(spin==1)
	{
		pin[0]=GPIO_PIN_12;
		pin[1]=GPIO_PIN_13;
		pin[2]=GPIO_PIN_14;
		pin[3]=GPIO_PIN_15;
		
	}
	else
	{
		pin[0]=GPIO_PIN_15;
		pin[1]=GPIO_PIN_14;
		pin[2]=GPIO_PIN_13;
		pin[3]=GPIO_PIN_12;
	}
	
}

void updateSpeed(void)
{
	speed=(speed + up + down)%10;
	if (speed<0){
		speed=0;
	}
	up=0;
	down=0;
}

void applySpeed(void)
{
	Tim3_Config();
}

void GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz??





/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
				select=1;
						break;	
			case GPIO_PIN_1:     //left button
				left=1;	
						break;
			case GPIO_PIN_2:    //right button						 
				right=1;
						break;
			case GPIO_PIN_3:    //up button
				up=1;
						break;
			case GPIO_PIN_5:    //down button						
				down=-1;		
						break;
			default://
						//default
						break;
	  } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
		
	
	
				
}

void Tim3_Config()
{
  /* Compute the prescaler value to have tim3 counter clock equal to 1 KHz */
  uint16_t PrescalerValue = (uint16_t)(SystemCoreClock / 5000) - 1;

  /* Set tim3 instance */
  Tim3_Handle.Instance = TIM3;

	if(speed==0){
		Tim3_Handle.Init.Period = speed;
	}
	else{
		if(full==1){
			Tim3_Handle.Init.Period = (5000*62)/speed;
		}
		else{
			Tim3_Handle.Init.Period = (5000*124)/speed;
		}
	}
  Tim3_Handle.Init.Prescaler = PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK) //the TIM_XXX_Start_IT function enable IT, and also enable Timer
  //so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

void TIM3_OC_Config(void)
{
	Tim3_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
	Tim3_OCInitStructure.Pulse = 5000;		//10000/10000 = 1s
	Tim3_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	
	HAL_TIM_OC_Init(&Tim3_Handle);
	
	HAL_TIM_OC_ConfigChannel(&Tim3_Handle,&Tim3_OCInitStructure,TIM_CHANNEL_1);
	
	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1);
					
}
 
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim )
{
  if (htim->Instance == TIM3)
  {
	move=1;
	BSP_LED_Toggle(LED4);
  }
}

static void Error_Handler(void)
{
  /* Turn LED4 on */
  //BSP_LED_On(LED4);
  while(1)
  {
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
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
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
