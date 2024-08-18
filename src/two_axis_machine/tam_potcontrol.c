#include "two_axis_machine/tam_potcontrol.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "two_axis_machine/tam_commands.h"

const TAM_PC_SpeedMapInterface TAM_PC_SPEEDMAP_INTERFACE_SIGMOID = {
  .createContext = (TAM_PC_CreateSpeedMapContextFunc_t)TAM_PC_Sigmoid_CreateSpeedMapContext,
  .mapPotValue = (TAM_PC_MapPotValueFunc_t)TAM_PC_Sigmoid_MapPotValueToSpeed
};

/**
 * @brief The ADC value filter context
 */
typedef struct {
  TAM_PC_FilterConfig config;         //!< The filter config
  uint32_t* pBuffer;                  //!< The filter value buffer (moving average window)
  uint32_t currentSum;                //!< Current sum of values in the buffer
  uint32_t numSamples;                //!< Current number of values in the buffer
  uint32_t currentIndex;              //!< Index the next value will be stored to in the buffer.
} TAM_PC_FilterContext;

/**
 * @brief Contains the context for the potentiometer control program
 */
typedef struct {
  TAM_PC_Config config;                       //!< The associated config
  TAM_Context *pTAMContext;                   //!< The two axis machine context

  TAM_PC_SpeedMapContext speedMapContextX;    //!< The speed mapper context for the x-axis 
  TAM_PC_SpeedMapContext speedMapContextY;    //!< The speed mapper context for the y-axis

  TAM_PC_FilterContext filterContextX;        //!< The value filter context for the x-axis
  TAM_PC_FilterContext filterContextY;        //!< The value filter context for the y-axis
  uint32_t lastCommandTime;                   //!< Timestamp of last motor command sent
  uint32_t potValueDiffTol;                   //!< Tolerance for potentiometer value change required to issue new motor commands

  uint32_t lastIssuedPotValX;                 //!< Last potentiometer value used for the speed command for the x-axis
  uint32_t lastIssuedPotValY;                 //!< Last potentiometer value used for the speed command for the y-axis
} TAM_PC_Context;

volatile TAM_PC_Context pcContext = {0};

TAM_Status TAM_PC_Filter_Init(TAM_PC_FilterConfig const * const pConfig, TAM_PC_FilterContext * const pContext);
void TAM_PC_Filter_SubmitInput(TAM_PC_FilterContext * const pContext, uint32_t newValue);
uint32_t TAM_PC_Filter_QueryOutput(TAM_PC_FilterContext const * const pContext);
TAM_Status TAM_PC_Filter_Deinit(TAM_PC_FilterContext * const pContext);

/**
 * @brief Callback for the ADC scan complete
 * 
 * @param potVals           The potentiometer values obtained from the ADC
 * @param resolutionBits    The ADC resolution
 * @param arg               The callback arg
 */
void TAM_PC_ADCScanCallback(volatile uint32_t const * const potVals, uint8_t resolutionBits, ADC_MCS_Scan_CallbackArg_t arg)
{
  TAM_PC_Context * const pCallbackContext = (TAM_PC_Context*)arg;

  const uint32_t potValX = potVals[0];
  const uint32_t potValY = potVals[1];

  LOG_DEBUG("Raw Pot values of x = %u and y = %u obtained", potValX, potValY);

  TAM_PC_Filter_SubmitInput(&(pCallbackContext->filterContextX), potValX);
  TAM_PC_Filter_SubmitInput(&(pCallbackContext->filterContextY), potValY);

  const uint32_t currentTime = HAL_GetTick();
  const uint32_t deltaTime = currentTime - pCallbackContext->lastCommandTime;
  if (deltaTime < pCallbackContext->config.minCommandPeriod)
  {
    return;
  }
  
  const uint32_t potValAvgX = TAM_PC_Filter_QueryOutput(&(pCallbackContext->filterContextX));
  const uint32_t potValAvgY = TAM_PC_Filter_QueryOutput(&(pCallbackContext->filterContextY));

  if (
    deltaTime < pCallbackContext->config.maxCommandPeriod &&
    abs(potValAvgX - pCallbackContext->lastIssuedPotValX) < pCallbackContext->potValueDiffTol &&
    abs(potValAvgY - pCallbackContext->lastIssuedPotValY) < pCallbackContext->potValueDiffTol
  )
  {
    return;
  }

  const float speedValX = TAM_PC_MapPotValueToSpeed(potValAvgX, &(pCallbackContext->speedMapContextX));
  const float speedValY = TAM_PC_MapPotValueToSpeed(potValAvgY, &(pCallbackContext->speedMapContextY));
   
  TAM_Run(pCallbackContext->pTAMContext, speedValX, speedValY);

  pCallbackContext->lastIssuedPotValX = potValAvgX;
  pCallbackContext->lastIssuedPotValY = potValAvgY;
  pCallbackContext->lastCommandTime = HAL_GetTick();
}

/**
 * @brief Initialize the potentiometer control program
 * 
 * @param pContext      The two axis machine context
 * @param pConfig       The potentiometer control config
 * @return TAM_Status   The two axis machine error status
 */
TAM_Status TAM_PC_Init(TAM_Context * const pContext, TAM_PC_Config const * const pConfig)
{
  TAM_PC_DeInit();

  memcpy(&pcContext.config, pConfig, sizeof(TAM_PC_Config));
  pcContext.config.speedMapConfigX.pMappingConfig = NULL;
  pcContext.config.speedMapConfigY.pMappingConfig = NULL;

  pcContext.pTAMContext = pContext;

  const uint8_t resolutionBits = ADC_GetResolutionBits(pConfig->adcResolution);

  TAM_PC_Filter_Init(&(pConfig->filterConfig), &pcContext.filterContextX);
  TAM_PC_Filter_Init(&(pConfig->filterConfig), &pcContext.filterContextY);

  pcContext.potValueDiffTol = pcContext.config.potValueDiffTolFactor * (1 << resolutionBits);

  pcContext.speedMapContextX.pMappingContext = malloc(pConfig->speedMapConfigX.contextSize);
  TAM_PC_CreateSpeedMapContext(resolutionBits, &pContext->xAxis.kinematicParams, &pConfig->speedMapConfigX, &pcContext.speedMapContextX);
  
  pcContext.speedMapContextY.pMappingContext = malloc(pConfig->speedMapConfigY.contextSize);
  TAM_PC_CreateSpeedMapContext(resolutionBits, &pContext->yAxis.kinematicParams, &pConfig->speedMapConfigY, &pcContext.speedMapContextY);

  const ADC_MCS_ChannelConfig potChannels[2] = {
    {
      .pin = {
        .port = pConfig->pinPotX.port,
        .pin = pConfig->pinPotX.pin,
      },
      .sampleTime = pConfig->sampleTime
    }, 
    {
      .pin = {
        .port = pConfig->pinPotY.port,
        .pin = pConfig->pinPotY.pin,
      },
      .sampleTime = pConfig->sampleTime
    }
  };

  const ADC_MCS_Config adcMcsConfig = {
    .acqMethod = pConfig->adcAcqMethod,
    .prescaler = ADC_CLOCKPRESCALER_PCLK_DIV4, 
    .resolution = pConfig->adcResolution,
    .numChannels = sizeof(potChannels) / sizeof(potChannels[0]),
    .channels = potChannels,
    .scanCallbackFunc = TAM_PC_ADCScanCallback,
    .scanCallbackArg = &pcContext
  };

  ADC_MCS_Init(&adcMcsConfig);

  LOG_INFO("Initialized ADC pot control for two axis machine");
  return TAM_OK;
}

/**
 * @brief Starts the potentiometer control program
 * 
 * @return TAM_Status   The two axis machine error status
 */
TAM_Status TAM_PC_Start()
{
  ASSERT_CHECK_LOG_RETURN(pcContext.pTAMContext != NULL, TAM_INVALID_OPERATION, "Pot control not yet initialized, can't start!")

  if (pcContext.config.adcAcqMethod == ADC_ACQ_DMA_CONTINUOUS)
  {
    ADC_MCS_Start_Continuous();
    while (1) {}
  }
  else
  {
    while (1)
    {
      ADC_MCS_Poll();
      HAL_Delay(1);
    }
  }
}

/**
 * @brief Deinitialize the potentiometer control program
 * 
 * @return TAM_Status   The two axis machine error status
 */
TAM_Status TAM_PC_DeInit()
{
  if (pcContext.speedMapContextX.pMappingContext != NULL)
  {
    free(pcContext.speedMapContextX.pMappingContext);
  }

  if (pcContext.speedMapContextY.pMappingContext != NULL)
  {
    free(pcContext.speedMapContextY.pMappingContext);
  }
  memset(&pcContext, 0, sizeof(TAM_PC_Context));

  return TAM_OK;
}

/**
 * @brief Creates the context for the sigmoid speed mapping algorithm from the config
 * 
 * @param resolutionBits  The resolution of the ADC value input
 * @param pKinParams      The kinematic parameters of the axis
 * @param pConfig         The sigmoid speed map config
 * @param pContext        The speed mapper context to initialize
 */
void TAM_PC_Sigmoid_CreateSpeedMapContext(
  uint8_t resolutionBits, 
  TAM_AxisKinematicParameters const * const pKinParams,
  TAM_PC_Sigmoid_SpeedMapConfig * const pConfig,
  TAM_PC_Sigmoid_SpeedMapContext * const pContext
)
{
  const float speedRange = pKinParams->maxSpeed - pKinParams->minSpeed;
  const float sigmoidEdgeSpeedTol = speedRange * pConfig->sigmoidEdgeSpeedTolFactor;

  pContext->sigmoidEdgeMinSpeed = pKinParams->minSpeed + sigmoidEdgeSpeedTol;
  pContext->sigmoidEdgeMaxSpeed = pKinParams->maxSpeed - sigmoidEdgeSpeedTol;

  pContext->valueMax = 1 << resolutionBits;
  pContext->distanceMax = pContext->valueMax / 2.0;

  pContext->sigmoidEdgeMin = pConfig->sigmoidEdgeMinValueFactor * pContext->distanceMax;
  pContext->sigmoidEdgeMax = pContext->distanceMax;

  pContext->sigmoidHorizontalShift = (pContext->sigmoidEdgeMin + pContext->sigmoidEdgeMax) / 2.0;
  pContext->sigmoidVerticalShift = pKinParams->minSpeed;

  pContext->sigmoidVerticalScale =  speedRange;
  pContext->sigmoidHorizontalScale = \
    log(
      pContext->sigmoidVerticalScale / (pContext->sigmoidEdgeMinSpeed - pContext->sigmoidVerticalShift) - 1
    ) / (pContext->sigmoidHorizontalShift - pContext->sigmoidEdgeMin);
}

/**
 * @brief Maps the potentiometer value to speed value using the sigmoid speed mapper algorithm
 * 
 * @param potVal      The potentiometer value to map
 * @param pContext    The sigmoid speed mapper context
 * @return float      The mapped speed value
 */
float TAM_PC_Sigmoid_MapPotValueToSpeed(
  uint32_t potVal, 
  TAM_PC_Sigmoid_SpeedMapContext const * const pContext
)
{
  const int32_t potCenterDisplacement = potVal - pContext->distanceMax;
  const uint32_t potCenterDistance = abs(potCenterDisplacement);

  if (potCenterDistance < pContext->sigmoidEdgeMin)
  {
    return 0;
  }

  const int speedSign = potCenterDisplacement > 0 ? 1 : -1;

  return speedSign * (pContext->sigmoidVerticalScale / 
    (1 + 
    exp(
      -pContext->sigmoidHorizontalScale * (potCenterDistance - pContext->sigmoidHorizontalShift)
    )) + pContext->sigmoidVerticalShift);
}

/**
 * @brief Create a speed map context from the generic config
 * 
 * @param resolutionBits  The ADC resolution in bits
 * @param pKinParams      The kinematic parameters of the axis
 * @param pConfig         The speed map config
 * @param pContext        The speed map context to initialize
 */
void TAM_PC_CreateSpeedMapContext(
  uint8_t resolutionBits, 
  TAM_AxisKinematicParameters const * const pKinParams,
  TAM_PC_SpeedMapConfig const * const pConfig,
  TAM_PC_SpeedMapContext * const pContext
)
{
  pContext->pMappingInterface = pConfig->pMappingInterface;
  pConfig->pMappingInterface->createContext(resolutionBits, pKinParams, pConfig->pMappingConfig, pContext->pMappingContext);
}

/**
 * @brief Maps the potentiometer value to speed value using the speed mapper algorithm
 * 
 * @param potVal      The potentiometer value to map
 * @param pContext    The speed mapper context
 * @return float      The mapped speed value
 */
float TAM_PC_MapPotValueToSpeed(
  uint32_t potVal, 
  TAM_PC_SpeedMapContext const * const pContext
)
{
  return pContext->pMappingInterface->mapPotValue(potVal, pContext->pMappingContext);
}

/**
 * @brief Initializes the ADC value filter context 
 * 
 * @param pConfig       The filter config
 * @param pContext      The filter context to initialize
 * @return TAM_Status   The two axis machine error status
 */
TAM_Status TAM_PC_Filter_Init(TAM_PC_FilterConfig const * const pConfig, TAM_PC_FilterContext * const pContext)
{
  TAM_PC_Filter_Deinit(pContext);
  memcpy(&(pContext->config), pConfig, sizeof(TAM_PC_FilterConfig));
  pContext->pBuffer = (uint32_t)malloc(sizeof(uint32_t) * pConfig->movingAvgSamples);
  memset(pContext->pBuffer, 0, sizeof(uint32_t) * pConfig->movingAvgSamples);
  return TAM_OK;
}

/**
 * @brief Submits a new value to the ADC value filter
 * 
 * @param pContext    The filter context 
 * @param newValue    The new value to submit into the filter
 */
void TAM_PC_Filter_SubmitInput(TAM_PC_FilterContext * const pContext, uint32_t newValue)
{
  pContext->currentSum -= pContext->pBuffer[pContext->currentIndex];
  pContext->pBuffer[pContext->currentIndex] = newValue;
  pContext->currentSum += newValue;
  pContext->currentIndex = (pContext->currentIndex + 1) % pContext->config.movingAvgSamples;
  if (pContext->numSamples < pContext->config.movingAvgSamples) 
  {
    pContext->numSamples++;
  }
}

/**
 * @brief Querys the output of the filter
 * 
 * @param pContext    The filter context 
 * @return uint32_t   The filtered output value
 */
uint32_t TAM_PC_Filter_QueryOutput(TAM_PC_FilterContext const * const pContext)
{
  if (pContext->numSamples == 0)
  {
    LOG_WARNING("Attempting to get filtered value but no values submitted to filter");
    return 0;
  }
  return pContext->currentSum / pContext->numSamples;
}

/**
 * @brief Deinitializes the ADC value filter
 * 
 * @param pContext      The filter context
 * @return TAM_Status   The two axis machine error status
 */
TAM_Status TAM_PC_Filter_Deinit(TAM_PC_FilterContext * const pContext)
{
  if (pContext->pBuffer != NULL)
  {
    free(pContext->pBuffer);
  }
  memset(pContext, 0, sizeof(TAM_PC_FilterContext));
  return TAM_OK;
}
