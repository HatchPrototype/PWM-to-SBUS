/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
void convertSBUS(void);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t timeout = 0;

uint16_t ch1_mmo[] = {0,0,0};
uint16_t ch2_mmo[] = {0,0,0};
uint16_t ch3_mmo[] = {0,0,0};
uint16_t ch4_mmo[] = {0,0,0};
uint16_t ch5_mmo[] = {0,0,0};
uint16_t ch6_mmo[] = {0,0,0};
uint16_t channels[] = {1500,1500,1000,1500,1000,1000};
uint8_t test[] = {0xFF,0xAA,0x21,0x11};
uint8_t SBUS[25];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim2);
  //HAL_UART_Transmit(&huart2,0xDE,1,500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    HAL_UART_Transmit(&huart2,test,4,0xFFFF);
    HAL_Delay(18);

    if(ch1_mmo[2]==1)
    {
      timeout = 0;
      uint16_t ch1_temp = 0;
      if(ch1_mmo[0]>ch1_mmo[1])
      {
        ch1_temp = (14000-ch1_mmo[0]) + (14000-ch1_mmo[1]);
      }
      else
      {
        ch1_temp = ch1_mmo[1] - ch1_mmo[0];
      }
      if((ch1_temp>=1000)&&(ch1_temp<=2000))
      {
        channels[0] = ch1_temp;
      }
      ch1_mmo[2] = 0;
    }


    if(ch2_mmo[2]==1)
    {
      timeout = 0;
      uint16_t ch2_temp = 0;
      if(ch2_mmo[0]>ch2_mmo[1])
      {
        ch2_temp = (14000-ch2_mmo[0]) + (14000-ch2_mmo[1]);
      }
      else
      {
        ch2_temp = ch2_mmo[1] - ch2_mmo[0];
      }
      if((ch2_temp>=1000)&&(ch2_temp<=2000))
      {
        channels[1] = ch2_temp;
      }
      ch2_mmo[2] = 0;
    }

    if(ch3_mmo[2]==1)
    {
      timeout = 0;
      uint16_t ch3_temp = 0;
      if(ch3_mmo[0]>ch3_mmo[1])
      {
        ch3_temp = (14000-ch3_mmo[0]) + (14000-ch3_mmo[1]);
      }
      else
      {
        ch3_temp = ch3_mmo[1] - ch3_mmo[0];
      }
      if((ch3_temp>=1000)&&(ch3_temp<=2000))
      {
        channels[2] = ch3_temp;
      }
      ch3_mmo[2] = 0;
    }

    if(ch4_mmo[2]==1)
    {
      timeout = 0;
      uint16_t ch4_temp = 0;
      if(ch4_mmo[0]>ch4_mmo[1])
      {
        ch4_temp = (14000-ch4_mmo[0]) + (14000-ch4_mmo[1]);
      }
      else
      {
        ch4_temp = ch4_mmo[1] - ch4_mmo[0];
      }
      if((ch4_temp>=1000)&&(ch4_temp<=2000))
      {
        channels[3] = ch4_temp;
      }
      ch4_mmo[2] = 0;
    }


    if(ch5_mmo[2]==1)
    {
      timeout = 0;
      uint16_t ch5_temp = 0;
      if(ch5_mmo[0]>ch5_mmo[1])
      {
        ch5_temp = (14000-ch5_mmo[0]) + (14000-ch5_mmo[1]);
      }
      else
      {
        ch5_temp = ch5_mmo[1] - ch5_mmo[0];
      }
      if((ch5_temp>=1000)&&(ch5_temp<=2000))
      {
        channels[4] = ch5_temp;
      }
      ch5_mmo[2] = 0;
    }

    if(ch6_mmo[2]==1)
    {
      timeout = 0;
      uint16_t ch6_temp = 0;
      if(ch6_mmo[0]>ch6_mmo[1])
      {
        ch6_temp = (14000-ch6_mmo[0]) + (14000-ch6_mmo[1]);
      }
      else
      {
        ch6_temp = ch6_mmo[1] - ch6_mmo[0];
      }
      if((ch6_temp>=1000)&&(ch6_temp<=2000))
      {
        channels[4] = ch6_temp;
      }
      ch6_mmo[2] = 0;
    }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance)
    {
        /* Toggle LED */


        if((ch1_mmo[2] == 0)&&(ch2_mmo[2] == 0)&&(ch3_mmo[2] == 0)&&(ch4_mmo[2] == 0)&&(ch5_mmo[2] == 0)&&(ch6_mmo[2] == 0))
        {
          timeout++;
        }
        if(timeout<5)
        {
          convertSBUS();
          HAL_UART_Transmit(&huart2, SBUS, 25,500);
        }


    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ( GPIO_Pin == GPIO_PIN_7)
    {

        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1)
        {

          ch1_mmo[0] = TIM2->CNT;
        }
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
        {
          ch1_mmo[1] = TIM2->CNT;
          ch1_mmo[2] = 1;
        }
    }


    else if ( GPIO_Pin == GPIO_PIN_6)
    {
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1)
        {
          ch2_mmo[0] = TIM2->CNT;
        }
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0)
        {
          ch2_mmo[1] = TIM2->CNT;
          ch2_mmo[2] = 1;
        }
    }


    else if ( GPIO_Pin == GPIO_PIN_4)
    {
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1)
        {
          ch3_mmo[0] = TIM2->CNT;
        }
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
        {
          ch3_mmo[1] = TIM2->CNT;
          ch3_mmo[2] = 1;
        }
    }


    else if ( GPIO_Pin == GPIO_PIN_3)
    {
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 1)
        {
          ch4_mmo[0] = TIM2->CNT;
        }
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
        {
          ch4_mmo[1] = TIM2->CNT;
          ch4_mmo[2] = 1;
        }
    }


    else if ( GPIO_Pin == GPIO_PIN_15)
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 1)
        {
          ch5_mmo[0] = TIM2->CNT;
        }
        else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0)
        {
          ch5_mmo[1] = TIM2->CNT;
          ch5_mmo[2] = 1;
        }
    }

    else if ( GPIO_Pin == GPIO_PIN_12)
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1)
        {
          ch6_mmo[0] = TIM2->CNT;
        }
        else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0)
        {
          ch6_mmo[1] = TIM2->CNT;
          ch6_mmo[2] = 1;
        }
    }


        // Write your code here
    }



  void convertSBUS(void)
  {
    uint16_t CH_sbus[23];

    for(int i=0;i<6;i++)
    {
      CH_sbus[i] = map(channels[i],1000,2000,0,2047);
    }
    for(int i=6;i<23;i++)
    {
      CH_sbus[i] = 1023;
    }

    SBUS[0] = 0xF0;
    SBUS[1] = (uint8_t) ((CH_sbus[0] & 0x07FF));
    SBUS[2] = (uint8_t) ((CH_sbus[0] & 0x07FF)>>8 | (CH_sbus[1] & 0x07FF)<<3);
		SBUS[3] = (uint8_t) ((CH_sbus[1] & 0x07FF)>>5 | (CH_sbus[2] & 0x07FF)<<6);
		SBUS[4] = (uint8_t) ((CH_sbus[2] & 0x07FF)>>2);
		SBUS[5] = (uint8_t) ((CH_sbus[2] & 0x07FF)>>10 | (CH_sbus[3] & 0x07FF)<<1);
		SBUS[6] = (uint8_t) ((CH_sbus[3] & 0x07FF)>>7 | (CH_sbus[4] & 0x07FF)<<4);
		SBUS[7] = (uint8_t) ((CH_sbus[4] & 0x07FF)>>4 | (CH_sbus[5] & 0x07FF)<<7);
		SBUS[8] = (uint8_t) ((CH_sbus[5] & 0x07FF)>>1);
		SBUS[9] = (uint8_t) ((CH_sbus[5] & 0x07FF)>>9 | (CH_sbus[6] & 0x07FF)<<2);
		SBUS[10] = (uint8_t) ((CH_sbus[6] & 0x07FF)>>6 | (CH_sbus[7] & 0x07FF)<<5);
		SBUS[11] = (uint8_t) ((CH_sbus[7] & 0x07FF)>>3);
		SBUS[12] = (uint8_t) ((CH_sbus[8] & 0x07FF));
		SBUS[13] = (uint8_t) ((CH_sbus[8] & 0x07FF)>>8 | (CH_sbus[9] & 0x07FF)<<3);
		SBUS[14] = (uint8_t) ((CH_sbus[9] & 0x07FF)>>5 | (CH_sbus[10] & 0x07FF)<<6);
		SBUS[15] = (uint8_t) ((CH_sbus[10] & 0x07FF)>>2);
		SBUS[16] = (uint8_t) ((CH_sbus[10] & 0x07FF)>>10 | (CH_sbus[11] & 0x07FF)<<1);
		SBUS[17] = (uint8_t) ((CH_sbus[11] & 0x07FF)>>7 | (CH_sbus[12] & 0x07FF)<<4);
		SBUS[18] = (uint8_t) ((CH_sbus[12] & 0x07FF)>>4 | (CH_sbus[13] & 0x07FF)<<7);
		SBUS[19] = (uint8_t) ((CH_sbus[13] & 0x07FF)>>1);
		SBUS[20] = (uint8_t) ((CH_sbus[13] & 0x07FF)>>9 | (CH_sbus[14] & 0x07FF)<<2);
		SBUS[21] = (uint8_t) ((CH_sbus[14] & 0x07FF)>>6 | (CH_sbus[15] & 0x07FF)<<5);
		SBUS[22] = (uint8_t) ((CH_sbus[15] & 0x07FF)>>3);
    SBUS[23] = 0x00;
    SBUS[24] = 0x00;
  }


  uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
