/**
  ******************************************************************************
  * @file    xnucleoihm02a1_interface.c
  * @brief   This file is used as interface between the 
  *          X-NUCLEO-IHM02A1 and the NUCLEO-F4xx board.
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

/* Includes ------------------------------------------------------------------*/
#include "xnucleoihm02a1/xnucleoihm02a1_interface.h"
#include "trace.h"

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @addtogroup X-NUCLEO-IHM02A1
  * @{
  */

/**
  * @addtogroup NUCLEO_Interface
  * @{
  */

/**
  * @addtogroup NUCLEO_Exported_Variables
  * @{
  */

/**
  * @brief  The data structure for all further instances to SPI1.
  */
SPI_HandleTypeDef hspi1;
/**
  * @brief  The data structure for all further instances to SPI2.
  */
SPI_HandleTypeDef hspi2;

/**
  * @}
  */ /* End of NUCLEO_Exported_Variables */

/**
  * @defgroup   NUCLEO_Private_Functions
  * @brief      NUCLEO Private Functions.
  * @{
  */


/**
  * @}
  */ /* End of NUCLEO_Private_Functions */

/**
  * @addtogroup NUCLEO_Private_Functions
  * @{
  */

/**
  * @brief  This function initializes the SPI1 MX
  *
  * @note   It sets the <i>hspi1</i> data structure for all further instances to
  *         SPI1
  *
  * @note   The SPI1 peripheral is configured as following:
  *         - Full-Duplex Master
  *         - 8-Bits
  *         - CPOL High
  *         - CPHA 2nd Edge
  *         - Baud Rate lower than 5 MBits/s
  */
void IHM02A1_SPI1_Init(void)
{
  #define MAX_BAUDRATE  5000000
  uint32_t freq;
  uint16_t freq_div;
  uint32_t spi_baudrateprescaler;
  
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  
  freq = HAL_RCC_GetPCLK2Freq();
  freq_div = (freq / MAX_BAUDRATE);
  
  if (freq_div < 2)
  {
    spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_2;
  }
  else
  {
    if (freq_div < 4)
    {
      spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_4;
    }
    else
    {
      if (freq_div < 8)
      {
        spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_8;
      }
      else
      {
        if (freq_div < 16)
        {
          spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_16;
        }
        else
        {
          if (freq_div < 32)
          {
            spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_32;
          }
          else
          {
            if (freq_div < 64)
            {
              spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_64;
            }
            else
            {
              if (freq_div < 128)
              {
                spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_128;
              }
              else
              {
                if (freq_div < 256)
                {
                  spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_256;
                }
                else
                {
                  /* the condition is not possible, you should reduce the CPU frequency */
                  while(1);
                }
              }
            }
          }
        }
      }
    }
  }
  
  hspi1.Init.BaudRatePrescaler = spi_baudrateprescaler;  // the baudrate will be lower than MAX_BAUDRATE (5 MBits/s)
  HAL_SPI_Init(&hspi1);
  
  LOG_INFO("SPI bus 1 initialized!");
}

/**
  * @brief  This function initializes the SPI2 MX
  *
  * @note   It sets the <i>hspi2</i> data structure for all further instances to
  *         SPI2
  *
  * @note   The SPI2 peripheral is configured as following:
  *         - Full-Duplex Master
  *         - 8-Bits
  *         - CPOL High
  *         - CPHA 2nd Edge
  *         - Baud Rate lower than 5 MBits/s
  */
void IHM02A1_SPI2_Init(void)
{
  #define MAX_BAUDRATE  5000000
  uint32_t freq;
  uint16_t freq_div;
  uint32_t spi_baudrateprescaler;
  
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  
  freq = HAL_RCC_GetPCLK1Freq();
  freq_div = (freq / MAX_BAUDRATE);
  
  if (freq_div < 2)
  {
    spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_2;
  }
  else
  {
    if (freq_div < 4)
    {
      spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_4;
    }
    else
    {
      if (freq_div < 8)
      {
        spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_8;
      }
      else
      {
        if (freq_div < 16)
        {
          spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_16;
        }
        else
        {
          if (freq_div < 32)
          {
            spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_32;
          }
          else
          {
            if (freq_div < 64)
            {
              spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_64;
            }
            else
            {
              if (freq_div < 128)
              {
                spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_128;
              }
              else
              {
                if (freq_div < 256)
                {
                  spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_256;
                }
                else
                {
                  /* the condition is not possible, you should reduce the CPU frequency */
                  while(1);
                }
              }
            }
          }
        }
      }
    }
  }
  
  hspi2.Init.BaudRatePrescaler = spi_baudrateprescaler; // the baudrate will be lower than MAX_BAUDRATE (5 MBits/s)
  HAL_SPI_Init(&hspi2);

  LOG_INFO("SPI bus 2 initialized!");
}

/**
  * @brief  Initialize the SPI used by the NUCLEO board.
  *
  * @note   It selects the @ref MX_SPI1_Init or @ref MX_SPI2_Init
  *         related to the defined macro @ref XNUCLEOIHM02A1_USE_SPI_1 or @ref XNUCLEOIHM02A1_USE_SPI_2.
  */
void IHM02A1_SPI_Init(void)
{
#ifdef XNUCLEOIHM02A1_USE_SPI_1
  LOG_INFO("Using SPI bus 1 for IHM02A1 interface");
  IHM02A1_SPI1_Init();
#endif
#ifdef XNUCLEOIHM02A1_USE_SPI_2
  LOG_INFO("Using SPI bus 2 for IHM02A1 interface");
  IHM02A1_SPI2_Init();
#endif

  LOG_INFO("SPI bus for IHM02A1 boards initialized!");
}

/**
  * @}
  */ /* End of NUCLEO_Private_Functions */

/**
  * @addtogroup NUCLEO_Exported_Functions
  * @{
  */

/**
  * @brief  This function initializes the board for the IHM02A1 including the SPI bus
  */
void IHM02A1_Init(void)
{
  IHM02A1_SPI_Init();
  BSP_Init();
  LOG_INFO("IHM02A1 expansion boards initialized!");
}

/**
  * @}
  */ /* End of NUCLEO_Exported_Functions */

/**
  * @}
  */ /* End of NUCLEO_Interface */

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1 */

/**
  * @}
  */ /* End of BSP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
