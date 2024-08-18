/**
 * @file adc_mcs.h
 * @brief HAL wrapper used to provide a convinient interface for running the ADC for multiple channels with the scan configuration
 */

#ifndef ADC_MCS_H
#define ADC_MCS_H

#include "gpio_defs.h"

typedef void* ADC_MCS_Channel_CallbackArg_t;
typedef void (*ADC_MCS_Channel_CallbackFunc_t)(GPIO_Pin const * const, uint32_t, uint8_t, ADC_MCS_Channel_CallbackArg_t);

typedef void* ADC_MCS_Scan_CallbackArg_t;
typedef void (*ADC_MCS_Scan_CallbackFunc_t)(volatile uint32_t const * const, uint8_t, ADC_MCS_Scan_CallbackArg_t);

/**
 * @brief Describes the method used to obtain data from the ADC
 */
typedef enum {
  ADC_ACQ_POLLING,        //!< Data is polled from the ADC manually by calling ADC_MCS_Poll
  ADC_ACQ_DMA_SINGLE,     //!< Data is obtained from the ADC using DMA for a single transfer (also requires calling ADC_MCS_Poll)
  ADC_ACQ_DMA_CONTINUOUS, //!< Data is obtained indefinitely from the ADC using DMA (started with call to ADC_MCS_Start_Continuous)
} ADC_AcquisitionMethod;

/**
 * @brief Configuration for a single channel to be used in the MCS system
 */
typedef struct {
  GPIO_Pin pin;                                 //!< The pin used by the analog device. Used to get the appropiate channel number
  uint32_t sampleTime;                          //!< The sample time used by the ADC (see samplingTime in ADC_ChannelConfTypeDef)
  ADC_MCS_Channel_CallbackFunc_t callbackFunc;  //!< Callback function called when the data for this channel is obtained
  ADC_MCS_Channel_CallbackArg_t callbackArg;    //!< Pointer to arbitrary argument to pass into the callback function
} ADC_MCS_ChannelConfig;

/**
 * @brief Configuration for the MCS system
 */
typedef struct {
  ADC_AcquisitionMethod acqMethod;              //!< The ADC acquisition method to configure for
  uint32_t prescaler;                           //!< The ADC prescaler value (refer to ADC_InitTypeDef)
  uint32_t resolution;                          //!< The ADC resolution value (refer to ADC_InitTypeDef)
  uint32_t numChannels;                         //!< Number of channels to configure for
  ADC_MCS_ChannelConfig* channels;              //!< Pointer to array of channel configurations
  ADC_MCS_Scan_CallbackFunc_t scanCallbackFunc; //!< Callback called when a scan is performed for all channels
  ADC_MCS_Scan_CallbackArg_t scanCallbackArg;   //!< Pointer to arbitrary argument to pass into the scan callback function
} ADC_MCS_Config;

/**
 * @brief The MCS context used to track the ADC, DMA and channel instances
 */
typedef struct {
  ADC_AcquisitionMethod acqMethod;              //!< The ADC acquisition method used
  ADC_HandleTypeDef hadc;                       //!< The ADC handle
  DMA_HandleTypeDef hdma;                       //!< The DMA handle
  uint32_t numChannels;                         //!< Number of channels used
  uint8_t resolutionBits;                       //!< ADC resolution number of bits 
  ADC_MCS_ChannelConfig channels[16];           //!< The ADC acquisition method to configure 
  volatile uint32_t adcValBuffer[16];           //!< Buffer used for storing the ADC values obtained by via DMA
  ADC_MCS_Scan_CallbackFunc_t scanCallbackFunc; //!< Callback called when a scan is performed for all channels
  ADC_MCS_Scan_CallbackArg_t scanCallbackArg;   //!< Pointer to arbitrary argument to pass into the scan callback function
} ADC_MCS_Context;

uint32_t ADC_GetPinChannel(GPIO_Pin const * const pPin);
uint8_t ADC_GetResolutionBits(uint32_t resolution);

void ADC_MCS_Init(ADC_MCS_Config const * const pConfig);
void ADC_MCS_Start_Continuous();
void ADC_MCS_Poll();

#endif