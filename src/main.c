/**
 ******************************************************************************
 * File Name          : main.c
 * Date               : 09/10/2014 11:13:03
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2014 STMicroelectronics
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

#include "stdlib.h"
#include "string.h"

#include "stm32f4xx_it.h"

#include "nucleo_board.h" // Must be included first to configure project defines
#include "gpio_utils.h"

#include "xnucleoihm02a1/example.h"
#include "xnucleoihm02a1/example_usart.h"

#include "synchronization_test_example.h"

#include "two_axis_machine/two_axis_machine.h"

#include "two_axis_machine/tam_potcontrol.h"
#include "two_axis_machine/tam_usart_commands.h"

/**
 * @defgroup   MotionControl
 * @{
 */

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @}
  */ /* End of BSP */

/**
 * @addtogroup MicrosteppingMotor_Example
 * @{
 */

/**
 * @defgroup   ExampleTypes
 * @{
 */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
#error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
#error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
#error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif


// Lab 4 exercise 4.1 and 4.2
void LED_Synchronization_Example()
{
  const InputOutputPins pins = 
  {
    .input_pin = 
    {
      .port = GPIOA,
      .pin = GPIO_PIN_1,
    },
    .output_pin = 
    {
      .port = GPIOA,
      .pin = GPIO_PIN_0,
    }
  };

  // Exercise 4.1
  // input_controlled_output_polling_loop(&pins);

  // Exercise 4.2
  input_controlled_output_interrupt(&pins);
}

void Microstepping_Example()
{
  IHM02A1_USART_TxWelcomeMessage();
  PRINTLN("X-CUBE-SPN2 v1.0.0");

#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  IHM02A1_MicrosteppingMotor_Example_01();

  /* Infinite loop */
  while (1)
    ;
#elif defined(MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  IHM02A1_USART_Fill_L6470_DaisyChainMnemonic();

  /*Initialize the motor parameters */
  IHM02A1_USART_Motor_Param_Reg_Init();

  /* Infinite loop */
  while (1)
  {
    /* Check if any Application Command for L6470 has been entered by USART */
    IHM02A1_USART_CheckAppCmd();
  }
#endif
}

TAM_Status TAM_UsartControl(TAM_Context * const pContext)
{
  while (1)
  {
    TAM_CHECK_STATUS_RETURN(TAM_USART_CheckAppCmd(&huart, pContext));
  }
}


TAM_Status TAM_PotControl(TAM_Context * const pContext)
{
  const TAM_PC_Sigmoid_SpeedMapConfig mappingConfig = {
    .sigmoidEdgeMinValueFactor = 0.1,
    .sigmoidEdgeSpeedTolFactor = 0.02
  };

  const TAM_PC_Config config = {
    .adcAcqMethod = ADC_ACQ_DMA_CONTINUOUS,
    .adcResolution = ADC_RESOLUTION_12B,
    .sampleTime = ADC_SAMPLETIME_3CYCLES,
    .pinPotX = {
      .port = GPIOA,
      .pin = GPIO_PIN_0,
    },
    .pinPotY = {
      .port = GPIOC,
      .pin = GPIO_PIN_2,
    },
    .filterConfig = {
      .movingAvgSamples = 10
    },
    .minCommandPeriod = 50,
    .maxCommandPeriod = 1000,
    .potValueDiffTolFactor = 0.05,
    .speedMapConfigX = {
      .pMappingInterface = &TAM_PC_SPEEDMAP_INTERFACE_SIGMOID,
      .pMappingConfig = &mappingConfig,
      .contextSize = sizeof(TAM_PC_Sigmoid_SpeedMapContext)
    },
    .speedMapConfigY = {
      .pMappingInterface = &TAM_PC_SPEEDMAP_INTERFACE_SIGMOID,
      .pMappingConfig = &mappingConfig,
      .contextSize = sizeof(TAM_PC_Sigmoid_SpeedMapContext)
    }
  };

  TAM_CHECK_STATUS_LOG_RETURN(TAM_PC_Init(pContext, &config), "Error initializing two axis machine potentiometer control");
  TAM_CHECK_STATUS_LOG_RETURN(TAM_PC_Start(), "Error starting two axis machine potentiometer control");

  return TAM_OK;
}

TAM_Status TAM_Main()
{
  /* Two Axis Machine initialization */
  TAM_CHECK_STATUS_LOG_RETURN(TAM_Init(), "Error initializing two axis machine");

  /* Two Axis Machine Control */
  //TAM_CHECK_STATUS_LOG_RETURN(TAM_UsartControl(&tamContext), "Error starting two axis machine USART control");
  TAM_CHECK_STATUS_LOG_RETURN(TAM_PotControl(&tamContext), "Error starting two axis machine potentiometer control");

  return TAM_OK;
}

/**
 * @}
  */ /* End of ExampleTypes */

/**
 * @brief The FW main module
 */
int main(void)
{
  /* NUCLEO board initialization */
  NB_Init();

  /* Two Axis Machine Program */
  TAM_Main();

  while (1)
  {
    
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @}
  */ /* End of MicrosteppingMotor_Example */

/**
 * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
