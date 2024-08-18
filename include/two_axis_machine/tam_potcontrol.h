/**
 * @file tam_potcontrol.h
 * @brief Core interface for controlling two axis machine with a set of potentiometers
 */

#ifndef TAM_POT_CONTROL_H
#define TAM_POT_CONTROL_H

#include "tam_context.h"
#include "adc_mcs.h"


/* CORE SPEED MAPPING INTERFACE */
typedef void (*TAM_PC_CreateSpeedMapContextFunc_t)(uint8_t resolutionBits, 
                          const TAM_AxisKinematicParameters * const pKinParams, 
                          const void * const pConfig, 
                          void * const pContext);
  
typedef float (*TAM_PC_MapPotValueFunc_t)(uint32_t potVal, const void * const pContext);

/**
 * @brief Defines a speed mapping interface through pointers to speed mapper functions.
 */
typedef struct {
    TAM_PC_CreateSpeedMapContextFunc_t createContext;     //!< The interface create context function 
    TAM_PC_MapPotValueFunc_t mapPotValue;                 //!< The interface value mapping function
} TAM_PC_SpeedMapInterface;

/**
 * @brief Config parameters for the potentiometer speed mapping
 */
typedef struct {
  const TAM_PC_SpeedMapInterface * pMappingInterface;     //!< The mapping interface for the mapper algorithm
  const void * pMappingConfig;                            //!< The config parameters for the mapper algorithm
  size_t contextSize;                                     //!< The size of the associated mapper context struct
} TAM_PC_SpeedMapConfig;

/**
 * @brief The speed mapping context
 */
typedef struct {
  const TAM_PC_SpeedMapInterface * pMappingInterface;     //!< The mapping interface for the mapper algorithm
  void * pMappingContext;                                 //!< The mapper context
} TAM_PC_SpeedMapContext;

void TAM_PC_CreateSpeedMapContext(
  uint8_t resolutionBits, 
  TAM_AxisKinematicParameters const * const pKinParams,
  TAM_PC_SpeedMapConfig const * const pConfig,
  TAM_PC_SpeedMapContext * const pContext
);

float TAM_PC_MapPotValueToSpeed(
  uint32_t potVal, 
  TAM_PC_SpeedMapContext const * const pContext
);

/* CORE FILTERING INTERFACE */

/**
 * @brief Config parameters for the ADC filter
 */
typedef struct {
  uint32_t movingAvgSamples;                            //!< The number of samples to average
} TAM_PC_FilterConfig;

/* CORE POTENTIOMETER CONTROL INTERFACE */

/**
 * @brief Config parameters for the potentiometer controller program
 */
typedef struct {
  // ADC Config
  ADC_AcquisitionMethod adcAcqMethod;                 //!< The ADC acquitistion method used by the potentiometers
  uint32_t adcResolution;                             //!< The ADC resolution
  uint32_t sampleTime;                                //!< The ADC sample time

  // Potentiometer pins
  GPIO_Pin pinPotX;                                   //!< The GPIO pin connected to the x-axis potentiometer
  GPIO_Pin pinPotY;                                   //!< The GPIO pin connected to the y-axis potentiometer
  
  // Post-processing config
  TAM_PC_FilterConfig filterConfig;                   //!< ADC value filter config
  uint32_t minCommandPeriod;                          //!< Minimum time that must pass before motor commands can be issued again
  uint32_t maxCommandPeriod;                          //!< The motor driver is updated with most recent speed value after this time
  float potValueDiffTolFactor;                        //!< Percent of the potentiometer travel in each direction to cause new command to be sent

  // Speed mapping configs - maps adc value to speed
  TAM_PC_SpeedMapConfig speedMapConfigX;              //!< Mapper config for the x-axis
  TAM_PC_SpeedMapConfig speedMapConfigY;              //!< Mapper config for the y-axis
} TAM_PC_Config;

TAM_Status TAM_PC_Init(TAM_Context * const pContext, TAM_PC_Config const * const pConfig);
TAM_Status TAM_PC_Start();
TAM_Status TAM_PC_DeInit();

/* MAPPING ALGORITHM IMPLEMENTATIONS */

/* SIGMOID MAPPING */

/**
 * @brief The speed mapper config for a sigmoid based mapper algorithm
 */
typedef struct {
  float sigmoidEdgeSpeedTolFactor;                  //!< Percent of speed difference used as sigmoid curve fitting tolerance 
  float sigmoidEdgeMinValueFactor;                  //!< Percent of potentiometer travel near center where speed is 0
} TAM_PC_Sigmoid_SpeedMapConfig;

/**
 * @brief The mapper context for a sigmoid based mapper algorithm
 */
typedef struct {
  float valueMax;                                 //!< Maximum input value (given by adc resolution)
  float distanceMax;                              //!< Maximum distance from potentiometer center
    
  float sigmoidEdgeMin;                           //!< Minimum distance mapped to sigmoid curve
  float sigmoidEdgeMinSpeed;                      //!< Speed at minimum sigmoid point
 
  float sigmoidEdgeMax;                           //!< Maximum distance mapped to sigmoid curve
  float sigmoidEdgeMaxSpeed;                      //!< Speed at maximum sigmoid point

  // Sigmoid curve parameters
  float sigmoidHorizontalShift;                   //!< The horizontal shift of the sigmoid curve
  float sigmoidVerticalShift;                     //!< The vertical shift of the sigmoid curve
  float sigmoidVerticalScale;                     //!< The vertical scaling factor of the sigmoid curve
  float sigmoidHorizontalScale;                   //!< The horizontal scaling factor of the sigmoid curve
} TAM_PC_Sigmoid_SpeedMapContext;

void TAM_PC_Sigmoid_CreateSpeedMapContext(
  uint8_t resolutionBits, 
  TAM_AxisKinematicParameters const * const pKinParams,
  TAM_PC_Sigmoid_SpeedMapConfig * const pConfig,
  TAM_PC_Sigmoid_SpeedMapContext * const pContext
);

float TAM_PC_Sigmoid_MapPotValueToSpeed(
  uint32_t potVal, 
  TAM_PC_Sigmoid_SpeedMapContext const * const pContext
);

extern const TAM_PC_SpeedMapInterface TAM_PC_SPEEDMAP_INTERFACE_SIGMOID;

#endif