#include "two_axis_machine/two_axis_machine.h"
#include "stddef.h"
#include "string.h"
#include "trace.h"
#include "switch_debounce.h"
#include "xnucleoihm02a1/xnucleoihm02a1_interface.h"

#define LIM_SWT_BACK_OFF_DISTANCE 1 // in cm

TAM_Context tamContext;

/**
 * @brief Numerical ids for limit switch types
 */
typedef enum {
  TAM_LIM_SWT_MIN = 1,
  TAM_LIM_SWT_MAX = -1
} TAM_Lim_Swt_Type;

/**
 * @brief Defines context for switch debounce callback parameter
 */
typedef struct {
  TAM_Axis axis;                            //!< The id of the axis limit switch was triggered on
  TAM_Axis_Lim_Swt_Config *pLimSwtConfig;   //!< The config of the limit switch triggered
  TAM_Lim_Swt_Type limSwtType;              //!< The limit switch type (min or max)
} TAM_Lim_Swt_EXTI_Handler_Context;

TAM_Lim_Swt_EXTI_Handler_Context limSwitchHandlerContext[4];

/**
 * @brief Gets the axis context from the axis ID
 * 
 * @param pContext                          The main two axis machine context
 * @param axis                              The axis ID
 * @return TAM_Axis_Context const* const    The corresponding axis context
 */
TAM_Axis_Context const * const TAM_GetAxisContext(TAM_Context const * const pContext, TAM_Axis axis)
{
  if (axis == TAM_AXIS_X)
  {
    return &(pContext->xAxis);
  }
  else if (axis == TAM_AXIS_Y)
  {
    return &(pContext->yAxis);
  }
  else
  {
    LOG_ERROR("Unknown Axis %d", axis);
    return NULL;
  }
}

/**
 * @brief Helper to get the friendly limit switch name from the switch type and axis for logging
 * 
 * @param axis          The axis type
 * @param limSwtType    The limit switch type
 * @return const char*  The friendly limit switch name for logging
 */
const char* TAM_GetLimitSwitchName(TAM_Axis axis, TAM_Lim_Swt_Type limSwtType)
{
  switch (axis)
  {
    case TAM_AXIS_X:
      switch (limSwtType)
      {
        case TAM_LIM_SWT_MIN: return "LIM_SWT_X_MIN";
        case TAM_LIM_SWT_MAX: return "LIM_SWT_X_MAX";
        default:
          LOG_ERROR("Invalid TAM limit switch type %u", limSwtType);
          return NULL;
      }
    case TAM_AXIS_Y:
      switch (limSwtType)
      {
        case TAM_LIM_SWT_MIN: return "LIM_SWT_Y_MIN";
        case TAM_LIM_SWT_MAX: return "LIM_SWT_Y_MAX";
        default:
          LOG_ERROR("Invalid TAM limit switch type %u", limSwtType);
          return NULL;
      }
    default:
      LOG_ERROR("Invalid TAM axis %u", axis);
      return NULL;
  }
}

/**
 * @brief OnTrigger callback for the limit switches called when the limit switch is initially triggered.
 * 
 * @param pPin      The GPIO pin of the limit switch that triggered the interrupt
 * @param newState  The new state of the limit switch
 * @param pArg      The callback arg
 */
void TAM_LimSwt_OnTrigger(GPIO_Pin const * const pPin, const GPIO_PinState newState, SWDB_CallbackArg_t pArg)
{
  TAM_Lim_Swt_EXTI_Handler_Context const * const pHandlerContext = (TAM_Lim_Swt_EXTI_Handler_Context*)pArg;
  LOG_INFO("Limit switch %s changed to %d", 
    TAM_GetLimitSwitchName(pHandlerContext->axis, pHandlerContext->limSwtType), newState
  );

  if (newState != pHandlerContext->pLimSwtConfig->activeState)
  {
    return;
  }

  // Completely stop the motors
  TAM_HardStop(&tamContext);
}

/**
 * @brief OnStabilize callback for the limit switches called when the limit switch has stopped bouncing.
 * 
 * @param pPin      The GPIO pin of the limit switch that triggered the interrupt
 * @param newState  The new state of the limit switch
 * @param pArg      The callback arg
 */
void TAM_LimSwt_OnStabilize(GPIO_Pin const * const pPin, const GPIO_PinState newState, SWDB_CallbackArg_t pArg)
{
  TAM_Lim_Swt_EXTI_Handler_Context const * const pHandlerContext = (TAM_Lim_Swt_EXTI_Handler_Context*)pArg;
  LOG_INFO("Limit switch %s settled to %d", 
    TAM_GetLimitSwitchName(pHandlerContext->axis, pHandlerContext->limSwtType), newState
  );

  if (newState != pHandlerContext->pLimSwtConfig->activeState)
  {
    return;
  }

  // Optionally Backoff the lim switch
  //TAM_Axis_Context const * const axisContext = TAM_GetAxisContext(&tamContext, pHandlerContext->axis);
  //TAM_AxisMove(&tamContext, pHandlerContext->axis, LIM_SWT_BACK_OFF_DISTANCE * pHandlerContext->limSwtType);
}

/**
 * @brief Initialize the limit switches including setting up interrupts
 * 
 * @return TAM_Status The two axis machine error status
 */
TAM_Status TAM_Init_Limit_Switch_Interrupts()
{
  LOG_DEBUG("Initializing interrupts for limit switches");

  const TAM_Axis_Lim_Swt_Config * LIM_SWT_CONFIGS[4] = {
    &TAM_CONFIG.xAxis.minLimSwtConfig,
    &TAM_CONFIG.xAxis.maxLimSwtConfig,
    &TAM_CONFIG.yAxis.minLimSwtConfig,
    &TAM_CONFIG.yAxis.maxLimSwtConfig
  };

  const TAM_Axis LIM_SWT_AXIS[4] = {TAM_AXIS_X, TAM_AXIS_X, TAM_AXIS_Y, TAM_AXIS_Y};
  const TAM_Lim_Swt_Type LIM_SWT_TYPE[4] = {TAM_LIM_SWT_MIN, TAM_LIM_SWT_MAX, TAM_LIM_SWT_MIN, TAM_LIM_SWT_MAX};

  memset(limSwitchHandlerContext, 0, sizeof(limSwitchHandlerContext));

  for (unsigned int i = 0; i < 4; ++i)
  {
    const TAM_Lim_Swt_EXTI_Handler_Context handlerContext = {
      .axis = LIM_SWT_AXIS[i],
      .pLimSwtConfig = LIM_SWT_CONFIGS[i],
      .limSwtType = LIM_SWT_TYPE[i],
    };

    memcpy(&(limSwitchHandlerContext[i]), &handlerContext, sizeof(TAM_Lim_Swt_EXTI_Handler_Context));

    SWDB_Init_EXTI(
      &(LIM_SWT_CONFIGS[i]->pin), 
      LIM_SWT_CONFIGS[i]->pull, 
      GPIO_IT_TRIGGER_MODE_CHANGE, 
      TAM_LimSwt_OnTrigger,
      TAM_LimSwt_OnStabilize,
      &(limSwitchHandlerContext[i]), 
      LIM_SWT_CONFIGS[i]->debounceStableTimeTol_ms,
      LIM_SWT_CONFIGS[i]->debounceUnstableTimeout_ms,
      &(LIM_SWT_CONFIGS[i]->extiConfig)
    );
  }

  LOG_INFO("Initialized limit swtich interrupts");

  return TAM_OK;
}

/**
 * @brief Deinitialize the limit switches 
 * 
 * @return TAM_Status The two axis machine error status
 */
void TAM_DeInit_Limit_Switch_Interrupts()
{
  LOG_DEBUG("Deinitializing limit swtich interrupts");

  TAM_Axis_Lim_Swt_Config const * const LIM_SWT_CONFIGS[4] = 
  {
    &TAM_CONFIG.xAxis.minLimSwtConfig,
    &TAM_CONFIG.xAxis.maxLimSwtConfig,
    &TAM_CONFIG.yAxis.minLimSwtConfig,
    &TAM_CONFIG.yAxis.maxLimSwtConfig
  };
  
  for (unsigned int i = 0; i < 4; ++i)
  {
    SWDB_DeInit(&(LIM_SWT_CONFIGS[i]->pin));
  }

  memset(limSwitchHandlerContext, 0, sizeof(limSwitchHandlerContext));

  LOG_DEBUG("Deinitialized limit swtich interrupts");
}

/**
 * @brief EXTI callback for the motor flags raised
 * 
 * @param pin   The pin ID the EXTI triggered on
 * @param arg   The callback arg
 */
void TAM_MotorFlagCallback(const GPIO_Pin_t pin, EXTI_CallbackArg_t arg)
{
  LOG_DEBUG("Motor flag raised!");

  // Check to see if any of the motors raised an important flag like step loss
  sL6470_StatusRegister_t motorStatus[2] = {0};
  TAM_GetMotorStatus(&tamContext, &motorStatus[0], &motorStatus[1]);

  // Print warnings for the raised flags
  // Normally you would "handle" these but out of scope for this project
  for (unsigned int motor = 0; motor < 2; ++motor)
  {
    if (motorStatus[motor].STEP_LOSS_A)
    {
      LOG_WARNING("Step loss detected in bridge A of motor %d", motor);
    }

    if (motorStatus[motor].STEP_LOSS_B)
    {
      LOG_WARNING("Step loss detected in bridge A of motor %d", motor);
    }

    if (motorStatus[motor].OCD)
    {
      LOG_WARNING("Overcurrent detected in motor %d", motor);
    }

    if (motorStatus[motor].TH_WRN)
    {
      LOG_WARNING("Thermal warning in motor %d", motor);
    }

    if (motorStatus[motor].TH_SD)
    {
      LOG_WARNING("Thermal shutdown in motor %d", motor);
    }

    if (motorStatus[motor].UVLO)
    {
      LOG_WARNING("Undervoltage lockout detected in motor %d", motor);
    }

    if (motorStatus[motor].WRONG_CMD)
    {
      LOG_WARNING("Wrong command detected in motor %d", motor);
    }

    if (motorStatus[motor].NOTPERF_CMD)
    {
      LOG_WARNING("Non-performable command detected in motor %d", motor);
    }
  }
}

/**
 * @brief EXTI callback for the user button (e-stop button)
 * 
 * @param pin   The EXTI pin the interrupt triggered on
 * @param arg   The callback arg
 */
void TAM_UserButtonCallback(const GPIO_Pin_t pin, EXTI_CallbackArg_t arg)
{
  LOG_INFO("Emergency stop invoked!");
  // This will block forever, as long as a higher priority interrupt doesn't trigger, NOTHING will execute
  BSP_EmergencyStop();
}

/**
 * @brief Initialize the two axis machine using the config in two_axis_machine.h
 * 
 * @return TAM_Status The two axis machine error status
 */
TAM_Status TAM_Init()
{
  LOG_DEBUG("Initializing two axis machine");

  /* X-NUCLEO-IHM02A1 initialization */
  IHM02A1_Init();

  TAM_CHECK_STATUS_LOG_RETURN(TAM_Create_Context(&TAM_CONFIG, &tamContext), "Failed to create TAM context");
  TAM_CHECK_STATUS_LOG_RETURN(TAM_SetDefaultParams(&tamContext), "Failed to set default TAM params");
  TAM_CHECK_STATUS_LOG_RETURN(TAM_Init_Limit_Switch_Interrupts(), "Failed to initialize limit switch interrupts");

  const EXTI_Config l6470NFlagExtiConfig = {.preemptPriority = PRIORITY_LEVEL_BELOW_NORMAL_DIAGNOSTIC, .subPriority = 0};
  EXTI_Register_Callback(L6470_nFLAG_GPIO_PIN, TAM_MotorFlagCallback, NULL, &l6470NFlagExtiConfig);

  const EXTI_Config eStopButtonExtiConfig = {.preemptPriority = PRIORITY_LEVEL_CRITICAL_SYSTEM, .subPriority = 0};
  EXTI_Register_Callback(USER_BUTTON_PIN, TAM_UserButtonCallback, NULL, &eStopButtonExtiConfig);

  LOG_INFO("Initialized two axis machine");

  return TAM_OK;
}

/**
 * @brief Deinitialize the two axis machine
 * 
 * @return TAM_Status The two axis machine error status
 */
TAM_Status TAM_DeInit()
{
  LOG_DEBUG("Deinitializing two axis machine");

  EXTI_Unregister_Callback(L6470_nFLAG_GPIO_PIN);
  TAM_DeInit_Limit_Switch_Interrupts();
  TAM_Delete_Context(&tamContext);

  LOG_INFO("Deinitialized two axis machine");

  return TAM_OK;
}
