#include "adc_mcs.h"
#include "stm32f4xx.h"
#include "trace.h"
#include "stdlib.h"
#include "string.h"
#include "error.h"


volatile ADC_MCS_Context adcContext = {0};

/**
 * @brief DMA 2 Stream 0 IRQ Handler
 */
void DMA2_Stream0_IRQHandler(void)
{
  if (adcContext.hdma.Instance != NULL)
  {
    HAL_DMA_IRQHandler(&adcContext.hdma);
  }
}

const uint32_t ADC_INVALID_CHANNEL = (uint32_t)-1;

/**
 * @brief Maps the GPIO pin to the corresponding ADC channel ID
 * 
 * @param pPin        Pointer to the GPIO_Pin struct containing the GPIO port and pin ID
 * @return uint32_t   The corresponding ADC channel ID
 */
uint32_t ADC_GetPinChannel(GPIO_Pin const * const pPin)
{
  if (pPin->port == GPIOA)
  {
    switch (pPin->pin)
    {
      case GPIO_PIN_0: return ADC_CHANNEL_0; 
      case GPIO_PIN_1: return ADC_CHANNEL_1;
      case GPIO_PIN_2: return ADC_CHANNEL_2;
      case GPIO_PIN_3: return ADC_CHANNEL_3;
      case GPIO_PIN_4: return ADC_CHANNEL_4;
      case GPIO_PIN_5: return ADC_CHANNEL_5;
      case GPIO_PIN_6: return ADC_CHANNEL_6;
      case GPIO_PIN_7: return ADC_CHANNEL_7;
      default:
        LOG_ERROR("Invalid GPIO pin %d in port A", pPin->pin);
        return ADC_INVALID_CHANNEL;
    }
  }
  
  if (pPin->port == GPIOB)
  {
    switch (pPin->pin)
    {
      case GPIO_PIN_0: return ADC_CHANNEL_8;
      case GPIO_PIN_1: return ADC_CHANNEL_9;
      default:
        LOG_ERROR("Invalid GPIO pin %d in port B", pPin->pin);
        return ADC_INVALID_CHANNEL;
    }
  }
  
  if (pPin->port == GPIOC)
  {
    switch (pPin->pin)
    {
      case GPIO_PIN_0: return ADC_CHANNEL_10;
      case GPIO_PIN_1: return ADC_CHANNEL_11;
      case GPIO_PIN_2: return ADC_CHANNEL_12;
      case GPIO_PIN_3: return ADC_CHANNEL_13;
      case GPIO_PIN_4: return ADC_CHANNEL_14;
      case GPIO_PIN_5: return ADC_CHANNEL_15;
      default:
        LOG_ERROR("Invalid GPIO pin %d in port C", pPin->pin);
        return ADC_INVALID_CHANNEL;
    }
  }
  
  LOG_ERROR("Invalid GPIO port for adc: %X", pPin->port);
  return ADC_INVALID_CHANNEL;
}

/**
 * @brief Maps the resolution ID used by the ADC into the number of bits
 * 
 * @param resolution  The resolution ID used by the ADC
 * @return uint8_t    The number of bits corresponding to the resolution ID
 */
uint8_t ADC_GetResolutionBits(uint32_t resolution)
{
  switch (resolution)
  {
  case ADC_RESOLUTION_6B: return 6;
  case ADC_RESOLUTION_8B: return 8;
  case ADC_RESOLUTION_10B: return 10;
  case ADC_RESOLUTION_12B: return 12;
  default:
    LOG_ERROR("Invalid ADC resolution %u", adcContext.resolutionBits);
    return 0;
  }
}

/**
 * @brief Initializes the ADC multi-channel scan system
 * 
 * @param pConfig   Pointer to the MCS config struct
 */
void ADC_MCS_Init(ADC_MCS_Config const * const pConfig)
{  
  LOG_DEBUG("Initializing ADC MCS");

  // Get the resolution bits
  adcContext.resolutionBits = ADC_GetResolutionBits(pConfig->resolution);
  ASSERT_CHECK_LOG_RETURN_VOID(adcContext.resolutionBits > 0, "Invalid ADC resolution %u", adcContext.resolutionBits);
  LOG_DEBUG("ADC MCS resolution set to %d bits", adcContext.resolutionBits);

  __HAL_RCC_ADC1_CLK_ENABLE();

  // Configure ADC peripheral
  adcContext.hadc.Instance = ADC1;
  adcContext.hadc.Init.ClockPrescaler = pConfig->prescaler;
  adcContext.hadc.Init.Resolution = pConfig->resolution;
  adcContext.hadc.Init.ScanConvMode = ENABLE;
  adcContext.hadc.Init.ContinuousConvMode = pConfig->acqMethod == ADC_ACQ_DMA_CONTINUOUS ? ENABLE : DISABLE;
  adcContext.hadc.Init.DiscontinuousConvMode = pConfig->acqMethod == ADC_ACQ_POLLING ? ENABLE : DISABLE;
  adcContext.hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adcContext.hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adcContext.hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adcContext.hadc.Init.NbrOfConversion = pConfig->numChannels;
  adcContext.hadc.Init.NbrOfDiscConversion = pConfig->acqMethod == ADC_ACQ_POLLING ? 1 : 0;
  adcContext.hadc.Init.DMAContinuousRequests = (pConfig->acqMethod == ADC_ACQ_DMA_CONTINUOUS || pConfig->acqMethod == ADC_ACQ_DMA_SINGLE) ? ENABLE : DISABLE;
  adcContext.hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  if (HAL_ADC_Init(&adcContext.hadc) != HAL_OK)
  {
    LOG_ERROR("Failed to initialize ADC");
    return;
  }

  LOG_DEBUG("Initialized core ADC peripheral");

  // Enable GPIO clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  LOG_DEBUG("Enabled GPIO clocks");

  // Configure DMA if needed
  if (pConfig->acqMethod == ADC_ACQ_DMA_CONTINUOUS || pConfig->acqMethod == ADC_ACQ_DMA_SINGLE)
  {
    __HAL_RCC_DMA2_CLK_ENABLE();
    adcContext.hdma.Instance = DMA2_Stream0;
    adcContext.hdma.Init.Channel = DMA_CHANNEL_0;
    adcContext.hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adcContext.hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    adcContext.hdma.Init.MemInc = DMA_MINC_ENABLE;
    adcContext.hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    adcContext.hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    adcContext.hdma.Init.Mode = DMA_CIRCULAR;
    adcContext.hdma.Init.Priority = DMA_PRIORITY_HIGH;
    adcContext.hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&adcContext.hdma) != HAL_OK)
    {
      LOG_ERROR("Failed to initialize DMA");
    }
    __HAL_LINKDMA(&adcContext.hadc, DMA_Handle, adcContext.hdma);

    // Configure the NVIC for DMA
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    LOG_INFO("Enabled DMA2 stream0 for ADC conversions");
  }

  adcContext.acqMethod = pConfig->acqMethod;
  adcContext.numChannels = pConfig->numChannels;
  memcpy(adcContext.channels, pConfig->channels, sizeof(ADC_MCS_ChannelConfig) * pConfig->numChannels);
  adcContext.scanCallbackFunc = pConfig->scanCallbackFunc;
  adcContext.scanCallbackArg = pConfig->scanCallbackArg;

  for (unsigned int i = 0; i < pConfig->numChannels; ++i)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pConfig->channels[i].pin.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(pConfig->channels[i].pin.port, &GPIO_InitStruct);

    ADC_ChannelConfTypeDef sConfig = {0};
    // Configure ADC channel
    sConfig.Channel = ADC_GetPinChannel(&(pConfig->channels[i].pin)); // TODO: Error check
    sConfig.Rank = i + 1;
    sConfig.SamplingTime = pConfig->channels[i].sampleTime;
    if (HAL_ADC_ConfigChannel(&adcContext.hadc, &sConfig) != HAL_OK)
    {
      LOG_ERROR("Failed to configure channel for pin %u", pConfig->channels[i].pin.pin);
      return;
    }
    LOG_INFO("Configured channel %u for pin %u as rank %u", sConfig.Channel, pConfig->channels[i].pin.pin, sConfig.Rank);
  }

  LOG_INFO("Initialized ADC for MCS");
}

/**
 * @brief DMA callback for ADC conversion complete
 * 
 * @param hadc The ADC handle
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == adcContext.hadc.Instance)
  {
    for (unsigned int i = 0; i < adcContext.numChannels; ++i)
    {
      if (adcContext.channels[i].callbackFunc != NULL)
      {
        adcContext.channels[i].callbackFunc(
          &adcContext.channels[i].pin,
          adcContext.adcValBuffer[i], 
          adcContext.resolutionBits,
          adcContext.channels[i].callbackArg
        );
      }
    }

    if (adcContext.scanCallbackFunc != NULL)
    {
      adcContext.scanCallbackFunc(adcContext.adcValBuffer, adcContext.resolutionBits, adcContext.scanCallbackArg);
    }
  }
}

/**
 * @brief Starts the MCS continuous DMA mode if enabled
 * 
 */
void ADC_MCS_Start_Continuous()
{
  if (adcContext.acqMethod != ADC_ACQ_DMA_CONTINUOUS)
  {
    LOG_ERROR("Continuous start should only be used for continous DMA acq method, use poll for others");
    return;
  }
  
  if (HAL_ADC_Start_DMA(&adcContext.hadc, adcContext.adcValBuffer, adcContext.numChannels) != HAL_OK)
  {
    LOG_ERROR("Failed to start DMA");
    return;
  }

  LOG_INFO("Started continous ADC MCS conversion with DMA");
}

/**
 * @brief Polls the ADC and calls the callback functions configured
 * 
 */
void ADC_MCS_Poll()
{
  if (adcContext.acqMethod == ADC_ACQ_POLLING)
  {
    for (unsigned int i = 0; i < adcContext.numChannels; ++i)
    {
      HAL_ADC_Start(&adcContext.hadc); // Start ADC Conversion
      if (HAL_ADC_PollForConversion(&adcContext.hadc, HAL_MAX_DELAY) == HAL_OK)
      {
        adcContext.adcValBuffer[i] = HAL_ADC_GetValue(&adcContext.hadc);
        if (adcContext.channels[i].callbackFunc != NULL)
        {
          adcContext.channels[i].callbackFunc(
            &adcContext.channels[i].pin,
            adcContext.adcValBuffer[i], 
            adcContext.resolutionBits,
            adcContext.channels[i].callbackArg
          );
        }
      }
    }

    HAL_ADC_Stop(&adcContext.hadc);

    if (adcContext.scanCallbackFunc != NULL)
    {
      adcContext.scanCallbackFunc(adcContext.adcValBuffer, adcContext.resolutionBits, adcContext.scanCallbackArg);
    }
  }
  else if (adcContext.acqMethod == ADC_ACQ_DMA_SINGLE)
  {
    HAL_ADC_Start_DMA(&adcContext.hadc, adcContext.adcValBuffer, adcContext.numChannels);
  }
  else if (adcContext.acqMethod == ADC_ACQ_DMA_CONTINUOUS)
  {
    LOG_ERROR("Polling should not be used for continous DMA");
  }
  else
  {
    LOG_ERROR("Unknown acquisition method %d", adcContext.acqMethod);
  }
}