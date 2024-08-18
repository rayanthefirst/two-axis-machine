#include "nucleo_board.h"
#include "stm32f4xx_nucleo.h"
#include "trace.h"
#include "config.h"

/**
  * @brief  This function configures the System Clock
  * 
  * @note   The System Clock will be configured as following:
  *         - PLL Source: HSI
  *         - SYSCLK: 84 MHz
  *         - HCLK: 84 MHz
  *         - APB1 Peripheral Clocks: 42 MHz
  *         - APB1 Timer Clocks: 84 MHz
  *         - APB2 Peripheral Clocks: 84 MHz
  *         - APB2 Timer Clocks: 84 MHz
  */
void NB_SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

}

/**
  * @brief  This function initializes the GPIO MX.
  */
void NB_GPIO_Init(void)
{
#ifdef NUCLEO_USE_USER_BUTTON
  /* Configures Button GPIO and EXTI Line */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#endif

#ifdef NUCLEO_USE_USER_LED
  /* Configures LED GPIO */
  BSP_LED_Init(LED2);
#endif
}

/**
  * @brief  This function initializes some peripherals of the NUCELO board
  *         (HAL, Clock, NVIC, LED and user button)
  */
void NB_Init(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  NB_SystemClock_Config();

  /* Initialize all board peripherals */
  NB_GPIO_Init();

  /* Initialize the trace utility */
  Trace_Init();

  LOG_INFO("Nucleo board initialized!");

#ifdef NUCLEO_USE_USER_LED
  /* Perform 3 repetition of blinking user LED at 50% duty cycle with 250 ms as period */
  NB_User_LED_Blinking(3, 750);
#endif
}


/**
  * @brief  Blinking user LED at 50% duty cycle.
  * @param  repetitions The number of  repetions.
  * @param  period_ms   The blinking period in ms.
  */
void NB_User_LED_Blinking(uint8_t repetitions, uint16_t period_ms)
{
  uint8_t r;
  uint16_t half_period_ms;
  
  half_period_ms = period_ms >> 1;
  
  for (r=0; r<repetitions; r++)
  {
    /* Switch on the user LED */
    BSP_LED_On(LED2);
    /* ms delay */
    HAL_Delay(half_period_ms);
    /* Switch off the user LED */
    BSP_LED_Off(LED2);
    /* ms delay */
    HAL_Delay(half_period_ms);
  }
}